#!/usr/bin/env python3
"""
EKF-SLAM con landmarks AprilTag + occupancy grid.

Diferencias clave vs tu nodo de localizacion:
- El estado crece: mu = [x, y, theta, m1x, m1y, m2x, m2y, ...]
  Cada tag es una variable estimada, no un dato fijo del yaml.
- Loop closure es implicito: al re-observar un tag ya mapeado, el update
  corrige robot Y todos los landmarks correlacionados via la covarianza.
- Data association es trivial porque usas tag_id (no hay ambiguedad).

Entradas:
- /odom_raw  -> dead reckoning para el predict (movimiento relativo en body frame)
- AprilTagDetectionArray + TF (base_footprint -> tag_<id>) para el update
- /scan      -> occupancy grid usando la pose corregida

Salidas:
- /map (OccupancyGrid)
- /slam/pose (PoseStamped)
- /slam/landmarks (MarkerArray)
- TF map->odom (opcional, default off para no pelear con tu localizacion)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException

from puzzlebot_interfaces.msg import AprilTagDetectionArray


def wrap(a):
    # normaliza angulo a [-pi, pi]
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class EKFSlam:
    """EKF-SLAM en numpy puro. Estado aumentado con landmarks."""

    def __init__(self):
        # estado inicial: solo robot en el origen
        self.mu = np.zeros(3)
        self.P = np.diag([1e-3, 1e-3, 1e-3])

        # tag_id -> indice de inicio en el estado (apunta a su x)
        self.lm_index = {}

        # ruido de movimiento (control). escala con la magnitud del movimiento
        self.k_trans = 0.15   # std por metro
        self.k_rot = 0.10     # std por radian
        self.q_base = 1e-4    # piso de ruido

        # ruido de observacion adaptativo (igual que tu nodo)
        self.sigma_base_sq = 0.0025  # 5cm std a 1m
        self.alpha = 0.01

        # gating chi2 95% con 2 DoF
        self.chi2_gate = 5.99

    def n(self):
        return self.mu.shape[0]

    def predict(self, dxb, dyb, dthb):
        # dxb, dyb, dthb es el movimiento relativo en el body frame del robot
        th = self.mu[2]
        c, s = math.cos(th), math.sin(th)

        # modelo de movimiento (solo afecta la pose del robot)
        self.mu[0] += c * dxb - s * dyb
        self.mu[1] += s * dxb + c * dyb
        self.mu[2] = wrap(self.mu[2] + dthb)

        # jacobiano del robot respecto a su pose
        Gx = np.eye(3)
        Gx[0, 2] = -s * dxb - c * dyb
        Gx[1, 2] = c * dxb - s * dyb

        # ruido de proceso solo en el bloque del robot
        trans = math.hypot(dxb, dyb)
        q_xy = (self.k_trans * trans) ** 2 + self.q_base
        q_th = (self.k_rot * abs(dthb) + self.k_trans * trans) ** 2 + self.q_base
        Q = np.diag([q_xy, q_xy, q_th])

        N = self.n()
        # F es identidad excepto el bloque 3x3 del robot
        F = np.eye(N)
        F[0:3, 0:3] = Gx

        self.P = F @ self.P @ F.T
        self.P[0:3, 0:3] += Q

    def _add_landmark(self, tag_id, z):
        # inicializa landmark nuevo con el modelo de observacion inverso
        x, y, th = self.mu[0], self.mu[1], self.mu[2]
        c, s = math.cos(th), math.sin(th)
        zx, zy = z

        mx = x + c * zx - s * zy
        my = y + s * zx + c * zy

        N = self.n()

        # jacobiano de la inicializacion respecto a la pose del robot (2x3)
        Gr = np.zeros((2, 3))
        Gr[0, 0] = 1.0
        Gr[0, 2] = -s * zx - c * zy
        Gr[1, 1] = 1.0
        Gr[1, 2] = c * zx - s * zy

        # jacobiano respecto a la observacion (2x2) = R(theta)
        Gz = np.array([[c, -s], [s, c]])

        # ruido de la observacion usada para inicializar
        d = math.hypot(zx, zy)
        r_var = self.sigma_base_sq + self.alpha * d * d
        Robs = np.eye(2) * r_var

        # Gr extendido a todo el estado (2 x N), no cero solo en columnas del robot
        Gr_full = np.zeros((2, N))
        Gr_full[:, 0:3] = Gr

        P_LL = Gr_full @ self.P @ Gr_full.T + Gz @ Robs @ Gz.T  # 2x2
        P_Lx = Gr_full @ self.P  # 2 x N

        # aumenta media
        self.mu = np.concatenate([self.mu, [mx, my]])

        # aumenta covarianza
        P_new = np.zeros((N + 2, N + 2))
        P_new[0:N, 0:N] = self.P
        P_new[N:N + 2, 0:N] = P_Lx
        P_new[0:N, N:N + 2] = P_Lx.T
        P_new[N:N + 2, N:N + 2] = P_LL
        self.P = P_new

        self.lm_index[tag_id] = N

    def update(self, tag_id, z):
        # si el tag es nuevo, lo inicializa y termina
        if tag_id not in self.lm_index:
            self._add_landmark(tag_id, z)
            return

        # loop closure implicito: re-observamos un tag ya mapeado
        idx = self.lm_index[tag_id]
        x, y, th = self.mu[0], self.mu[1], self.mu[2]
        mx, my = self.mu[idx], self.mu[idx + 1]
        c, s = math.cos(th), math.sin(th)

        dx = mx - x
        dy = my - y

        # observacion esperada en el frame del robot (igual que tu h_mu)
        z_pred = np.array([c * dx + s * dy,
                           -s * dx + c * dy])

        z = np.asarray(z, dtype=float)
        innov = z - z_pred

        # jacobiano H (2 x N): bloque del robot y bloque del landmark
        N = self.n()
        H = np.zeros((2, N))
        # respecto a la pose del robot
        H[0, 0] = -c
        H[0, 1] = -s
        H[0, 2] = -s * dx + c * dy
        H[1, 0] = s
        H[1, 1] = -c
        H[1, 2] = -c * dx - s * dy
        # respecto al landmark observado
        H[0, idx] = c
        H[0, idx + 1] = s
        H[1, idx] = -s
        H[1, idx + 1] = c

        # ruido adaptativo segun distancia
        d = math.hypot(z[0], z[1])
        r_var = self.sigma_base_sq + self.alpha * d * d
        R = np.eye(2) * r_var

        S = H @ self.P @ H.T + R

        # gating de Mahalanobis, descarta outliers
        mahal = float(innov.T @ np.linalg.solve(S, innov))
        if mahal > self.chi2_gate:
            return

        K = self.P @ H.T @ np.linalg.inv(S)
        self.mu = self.mu + K @ innov
        self.mu[2] = wrap(self.mu[2])

        # forma de Joseph para mantener simetria y semidefinida positiva
        I_KH = np.eye(N) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    def robot_pose(self):
        return self.mu[0], self.mu[1], self.mu[2]

    def landmarks(self):
        # devuelve dict tag_id -> (x, y)
        out = {}
        for tid, idx in self.lm_index.items():
            out[tid] = (self.mu[idx], self.mu[idx + 1])
        return out


class OccupancyMapper:
    """Occupancy grid por log-odds. Limpia el scan antes de integrar."""

    def __init__(self, res, width, height, origin_x, origin_y):
        self.res = res
        self.width = width    # celdas en x
        self.height = height  # celdas en y
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.log = np.zeros((height, width), dtype=np.float32)

        # parametros de log-odds
        self.l_occ = 0.85
        self.l_free = -0.4
        self.l_min = -4.0
        self.l_max = 4.0

    def _world_to_cell(self, wx, wy):
        col = int((wx - self.origin_x) / self.res)
        row = int((wy - self.origin_y) / self.res)
        return col, row

    def _in_bounds(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    def clean_scan(self, msg):
        # filtra NaN, inf, fuera de rango y speckle (puntos aislados)
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = ranges.shape[0]
        angles = msg.angle_min + np.arange(n) * msg.angle_increment

        valid = np.isfinite(ranges)
        valid &= ranges >= msg.range_min
        valid &= ranges <= msg.range_max

        # speckle: descarta beam si difiere mucho de ambos vecinos
        r = ranges.copy()
        speckle = np.zeros(n, dtype=bool)
        if n >= 3:
            dl = np.abs(r[1:-1] - r[:-2])
            dr = np.abs(r[1:-1] - r[2:])
            speckle[1:-1] = (dl > 0.3) & (dr > 0.3)
        valid &= ~speckle

        return ranges[valid], angles[valid]

    def integrate(self, x, y, th, ranges, angles, range_max):
        rc, rr = self._world_to_cell(x, y)
        if not self._in_bounds(rc, rr):
            return

        for rng, ang in zip(ranges, angles):
            ex = x + rng * math.cos(th + ang)
            ey = y + rng * math.sin(th + ang)
            ec, er = self._world_to_cell(ex, ey)

            # marca libres a lo largo del rayo (Bresenham)
            for cc, cr in self._bresenham(rc, rr, ec, er):
                if self._in_bounds(cc, cr):
                    self.log[cr, cc] = max(self.l_min, self.log[cr, cc] + self.l_free)

            # marca ocupado el extremo solo si fue un hit real
            if rng < range_max and self._in_bounds(ec, er):
                self.log[er, ec] = min(self.l_max, self.log[er, ec] + self.l_occ)

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        # celdas entre origen y extremo, sin incluir el extremo
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        cx, cy = x0, y0
        while True:
            if cx == x1 and cy == y1:
                break
            cells.append((cx, cy))
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy
        return cells

    def to_msg(self, stamp, frame_id):
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.info.resolution = self.res
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        # log-odds a probabilidad 0-100, -1 desconocido
        data = np.full((self.height, self.width), -1, dtype=np.int8)
        prob = 1.0 - 1.0 / (1.0 + np.exp(self.log))
        known = np.abs(self.log) > 1e-3
        data[known] = (prob[known] * 100).astype(np.int8)
        msg.data = data.flatten().tolist()
        return msg


class SlamNode(Node):
    def __init__(self):
        super().__init__("slam_ekf")

        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_resolution", 0.05)
        self.declare_parameter("map_cells", 400)       # mapa cuadrado
        self.declare_parameter("map_origin", -10.0)    # esquina inferior izq
        self.declare_parameter("scan_beam_step", 2)    # submuestreo de beams

        odom_topic = self.get_parameter("odom_topic").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.base_frame = self.get_parameter("base_frame").value
        res = self.get_parameter("map_resolution").value
        cells = self.get_parameter("map_cells").value
        origin = self.get_parameter("map_origin").value
        self.beam_step = self.get_parameter("scan_beam_step").value

        self.ekf = EKFSlam()
        self.mapper = OccupancyMapper(res, cells, cells, origin, origin)

        self.last_odom = None  # (x, y, theta) previo para el delta
        self.last_scan = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.create_subscription(
            AprilTagDetectionArray, "/apriltag/camera_pose",
            self.tag_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)

        self.pose_pub = self.create_publisher(PoseStamped, "/slam/pose", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 1)
        self.lm_pub = self.create_publisher(MarkerArray, "/slam/landmarks", 10)

        # publica mapa y pose a ritmo fijo
        self.create_timer(0.1, self.publish_pose)
        self.create_timer(1.0, self.publish_map)

        self.get_logger().info("EKF-SLAM corriendo")

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        cth = yaw_from_quat(q.x, q.y, q.z, q.w)

        if self.last_odom is None:
            self.last_odom = (cx, cy, cth)
            return

        px, py, pth = self.last_odom
        # delta de movimiento en el mundo de odom
        dxw = cx - px
        dyw = cy - py
        dth = wrap(cth - pth)
        # llevado al body frame del robot (independiente del frame global)
        c, s = math.cos(pth), math.sin(pth)
        dxb = c * dxw + s * dyw
        dyb = -s * dxw + c * dyw

        self.last_odom = (cx, cy, cth)

        # ignora deltas absurdos (saltos o reinicios)
        if math.hypot(dxb, dyb) > 1.0 or abs(dth) > 1.0:
            return

        self.ekf.predict(dxb, dyb, dth)

    def tag_cb(self, msg):
        stamp = msg.header.stamp
        for det in msg.detections:
            frame = "tag_" + str(det.tag_id)
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, frame, stamp,
                    timeout=rclpy.duration.Duration(seconds=0.1))
            except TransformException:
                continue

            zx = tf.transform.translation.x
            zy = tf.transform.translation.y

            # rechaza detecciones lejanas (ruidosas)
            if math.hypot(zx, zy) > 3.5:
                continue

            self.ekf.update(det.tag_id, (zx, zy))

    def scan_cb(self, msg):
        self.last_scan = msg

    def publish_pose(self):
        x, y, th = self.ekf.robot_pose()
        stamp = self.get_clock().now().to_msg()

        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = "map"
        ps.pose.position.x = x
        ps.pose.position.y = y
        qx, qy, qz, qw = quat_from_yaw(th)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)

        self.publish_landmarks(stamp)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = "map"
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    def publish_landmarks(self, stamp):
        arr = MarkerArray()
        for tid, (lx, ly) in self.ekf.landmarks().items():
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = "map"
            m.ns = "landmarks"
            m.id = int(tid)
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = lx
            m.pose.position.y = ly
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.r = 1.0
            m.color.g = 0.6
            m.color.a = 1.0
            arr.markers.append(m)
        self.lm_pub.publish(arr)

    def publish_map(self):
        if self.last_scan is None:
            return
        x, y, th = self.ekf.robot_pose()
        msg = self.last_scan

        ranges, angles = self.mapper.clean_scan(msg)
        # submuestrea para no saturar en python
        ranges = ranges[::self.beam_step]
        angles = angles[::self.beam_step]

        self.mapper.integrate(x, y, th, ranges, angles, msg.range_max)

        stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.mapper.to_msg(stamp, "map"))


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()