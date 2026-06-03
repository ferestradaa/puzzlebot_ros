import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import Buffer, TransformListener


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def angle_wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class DWALocalPlanner(Node):
    def __init__(self):
        super().__init__('dwa_local_planner')

        # limites cinematicos — conservadores para prueba inicial
        self.declare_parameter('v_max', 0.08)
        self.declare_parameter('w_max', 0.8)

        # rampas de aceleracion en la ventana dinamica
        self.declare_parameter('a_lin_up',   0.3)
        self.declare_parameter('a_lin_down', 0.5)
        self.declare_parameter('a_ang',      1.5)

        # rampa de salida — limita el delta real enviado al motor por ciclo
        # esto evita saturacion independientemente de lo que elija DWA
        self.declare_parameter('ramp_v', 0.04)   # m/s por ciclo maximo
        self.declare_parameter('ramp_w', 0.3)    # rad/s por ciclo maximo

        # muestreo de la ventana dinamica
        self.declare_parameter('v_samples', 10)
        self.declare_parameter('w_samples', 21)

        # horizonte de simulacion
        self.declare_parameter('sim_time', 2.0)
        self.declare_parameter('sim_dt',   0.1)

        # geometria y seguridad
        self.declare_parameter('robot_radius', 0.15)
        self.declare_parameter('clear_cap',    1.0)
        self.declare_parameter('scan_stride',  3)

        # seguimiento de path
        self.declare_parameter('lookahead', 0.4)
        self.declare_parameter('goal_tol',  0.08)

        # alineacion final
        self.declare_parameter('final_yaw_tol',  0.05)
        self.declare_parameter('final_yaw_gain', 0.5)

        # pesos de scoring
        self.declare_parameter('k_goal',   1.0)
        self.declare_parameter('k_clear',  0.6)
        self.declare_parameter('k_speed',  0.4)
        self.declare_parameter('k_smooth', 0.4)

        # frames y rate
        self.declare_parameter('map_frame',     'map')
        self.declare_parameter('base_frame',    'base_link')
        self.declare_parameter('sensor_frame',  'laser')   # frame del scan
        self.declare_parameter('rate',          15.0)

        g = self.get_parameter
        self.v_max          = g('v_max').value
        self.w_max          = g('w_max').value
        self.a_lin_up       = g('a_lin_up').value
        self.a_lin_down     = g('a_lin_down').value
        self.a_ang          = g('a_ang').value
        self.ramp_v         = g('ramp_v').value
        self.ramp_w         = g('ramp_w').value
        self.v_samples      = g('v_samples').value
        self.w_samples      = g('w_samples').value
        self.sim_time       = g('sim_time').value
        self.sim_dt         = g('sim_dt').value
        self.robot_radius   = g('robot_radius').value
        self.clear_cap      = g('clear_cap').value
        self.scan_stride    = g('scan_stride').value
        self.lookahead      = g('lookahead').value
        self.goal_tol       = g('goal_tol').value
        self.final_yaw_tol  = g('final_yaw_tol').value
        self.final_yaw_gain = g('final_yaw_gain').value
        self.k_goal         = g('k_goal').value
        self.k_clear        = g('k_clear').value
        self.k_speed        = g('k_speed').value
        self.k_smooth       = g('k_smooth').value
        self.map_frame      = g('map_frame').value
        self.base_frame     = g('base_frame').value
        self.sensor_frame   = g('sensor_frame').value

        rate     = g('rate').value
        self.dt  = 1.0 / rate
        self.n_steps = max(1, int(self.sim_time / self.sim_dt))

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path        = []
        self.goal_yaw    = None
        self.aligning    = False
        self.latest_scan = None

        # velocidades realmente enviadas al robot (para la rampa de salida)
        self.sent_v = 0.0
        self.sent_w = 0.0

        # velocidades reportadas por odometria (para la ventana dinamica)
        self.odom_v = 0.0
        self.odom_w = 0.0

        # QoS best effort depth 1 para el scan
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Path,      '/path', self.path_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, scan_qos)
        self.create_subscription(Odometry,  '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('dwa local planner listo')

    def path_cb(self, msg):
        self.path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.aligning = False
        if msg.poses:
            q = msg.poses[-1].pose.orientation
            if abs(q.w - 1.0) > 1e-3 or abs(q.z) > 1e-3:
                self.goal_yaw = yaw_from_quat(q)
            else:
                self.goal_yaw = None

    def scan_cb(self, msg):
        self.latest_scan = msg

    def odom_cb(self, msg):
        self.odom_v = msg.twist.twist.linear.x
        self.odom_w = msg.twist.twist.angular.z

    def get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            tr = t.transform.translation
            return tr.x, tr.y, yaw_from_quat(t.transform.rotation)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return None

    def get_obstacles_base_link(self):
        """
        Transforma el scan a base_link.
        Los obstaculos quedan en coordenadas del robot — ground truth de deteccion
        independiente del ruido de localizacion en el mapa.
        """
        scan = self.latest_scan
        if scan is None:
            return None

        # transform del frame del sensor a base_link (extrinseca fija, no depende de odom/map)
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, scan.header.frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return None

        tx  = t.transform.translation.x
        ty  = t.transform.translation.y
        yaw = yaw_from_quat(t.transform.rotation)

        ranges = np.array(scan.ranges[::self.scan_stride], dtype=np.float64)
        n      = ranges.shape[0]
        angles = (scan.angle_min
                  + np.arange(n) * scan.angle_increment * self.scan_stride)

        valid  = (np.isfinite(ranges)
                  & (ranges >= scan.range_min)
                  & (ranges <= scan.range_max))
        ranges = ranges[valid]
        angles = angles[valid]

        if ranges.shape[0] == 0:
            return np.empty((0, 2))

        # coordenadas en frame del sensor
        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)

        # rotar y trasladar a base_link
        bx = tx + math.cos(yaw) * lx - math.sin(yaw) * ly
        by = ty + math.sin(yaw) * lx + math.cos(yaw) * ly
        return np.stack([bx, by], axis=1)

    def nearest_index(self, rx, ry, robot_yaw):
        """
        Busca el punto mas cercano en el path expresado en base_link.
        rx, ry son las coordenadas del robot en map (para convertir el path).
        Devuelve el indice en self.path.
        """
        best_i, best_d = 0, float('inf')
        for i, (px, py) in enumerate(self.path):
            # distancia en map — suficiente para encontrar el mas cercano
            d = (px - rx) ** 2 + (py - ry) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def lookahead_target_base(self, robot_x, robot_y, robot_yaw):
        """
        Devuelve el punto lookahead en coordenadas de base_link.
        """
        near = self.nearest_index(robot_x, robot_y, robot_yaw)
        for i in range(near, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - robot_x, py - robot_y) >= self.lookahead:
                # convertir a base_link
                dx = px - robot_x
                dy = py - robot_y
                c, s = math.cos(-robot_yaw), math.sin(-robot_yaw)
                return c * dx - s * dy, s * dx + c * dy
        # ultimo punto del path en base_link
        px, py = self.path[-1]
        dx = px - robot_x
        dy = py - robot_y
        c, s = math.cos(-robot_yaw), math.sin(-robot_yaw)
        return c * dx - s * dy, s * dx + c * dy

    def dynamic_window(self):
        # usa velocidades de odometria como estado actual
        v_lo = max(0.0,        self.odom_v - self.a_lin_down * self.dt)
        v_hi = min(self.v_max, self.odom_v + self.a_lin_up   * self.dt)
        w_lo = max(-self.w_max, self.odom_w - self.a_ang * self.dt)
        w_hi = min( self.w_max, self.odom_w + self.a_ang * self.dt)

        vs = np.linspace(v_lo, v_hi, self.v_samples)
        ws = np.linspace(w_lo, w_hi, self.w_samples)
        vv, ww = np.meshgrid(vs, ws)
        return vv.ravel(), ww.ravel()

    def rollout(self, vs, ws):
        """
        Rollout en frame local del robot (parte de x=0, y=0, th=0).
        Esto es correcto porque los obstaculos ya estan en base_link.
        """
        T   = self.n_steps
        ks  = np.arange(1, T + 1)
        dt  = self.sim_dt
        v   = vs[:, None]
        w   = ws[:, None]

        th       = w * ks * dt                      # th0 = 0
        straight = np.abs(w) < 1e-4
        w_safe   = np.where(straight, 1.0, w)

        x_arc = (v / w_safe) * np.sin(th)           # x0=0, sin(th0)=0
        y_arc = (v / w_safe) * (1.0 - np.cos(th))   # y0=0, cos(th0)=1 → -cos+1
        x_str = v * ks * dt
        y_str = np.zeros_like(x_str)

        x = np.where(straight, x_str, x_arc)
        y = np.where(straight, y_str, y_arc)
        return x, y, th

    def clearances(self, xs, ys, obs):
        N, T = xs.shape
        if obs.shape[0] == 0:
            return np.full(N, np.inf)
        pts        = np.stack([xs.ravel(), ys.ravel()], axis=1)
        d          = np.sqrt(((pts[:, None, :] - obs[None, :, :]) ** 2).sum(axis=2))
        min_per_pt = d.min(axis=1)
        return min_per_pt.reshape(N, T).min(axis=1)

    @staticmethod
    def norm(a, mask):
        vals = a[mask]
        lo, hi = vals.min(), vals.max()
        if hi - lo < 1e-6:
            return np.ones_like(a)
        return (a - lo) / (hi - lo)

    def control_loop(self):
        pose = self.get_pose()
        if pose is None or len(self.path) < 2:
            self.publish(0.0, 0.0)
            return

        rx, ry, ryaw = pose
        gx, gy       = self.path[-1]
        dist_to_goal = math.hypot(gx - rx, gy - ry)

        if self.aligning:
            if self.goal_yaw is None:
                self.publish(0.0, 0.0)
                self.path     = []
                self.aligning = False
                return
            yaw_err = angle_wrap(self.goal_yaw - ryaw)
            if abs(yaw_err) < self.final_yaw_tol:
                self.publish(0.0, 0.0)
                self.path     = []
                self.aligning = False
                self.get_logger().info('meta alcanzada con orientacion final')
                return
            w = clamp(self.final_yaw_gain * yaw_err, -self.w_max, self.w_max)
            self.publish(0.0, w)
            return

        if dist_to_goal < self.goal_tol:
            self.publish(0.0, 0.0)
            if self.goal_yaw is not None:
                self.aligning = True
            else:
                self.path = []
                self.get_logger().info('meta alcanzada')
            return

        obs = self.get_obstacles_base_link()
        if obs is None:
            self.publish(0.0, 0.0)
            return

        # target lookahead en base_link (coordenadas locales del robot)
        tgt_x, tgt_y = self.lookahead_target_base(rx, ry, ryaw)

        vs, ws          = self.dynamic_window()
        xs, ys, ths     = self.rollout(vs, ws)
        clr             = self.clearances(xs, ys, obs)

        valid = clr >= self.robot_radius
        if not np.any(valid):
            # recovery: gira hacia el target
            bearing = math.atan2(tgt_y, tgt_x)  # ya en base_link, th0=0
            w = clamp(math.copysign(0.4, bearing), -self.w_max, self.w_max)
            self.publish(0.0, w)
            self.get_logger().warn('sin trayectoria libre, recovery girando')
            return

        end_x = xs[:, -1]
        end_y = ys[:, -1]

        to_target  = np.hypot(end_x - tgt_x, end_y - tgt_y)
        goal_raw   = -to_target
        clear_raw  = np.minimum(clr, self.clear_cap)
        speed_raw  = vs.copy()
        smooth_raw = -np.abs(ws - self.odom_w)

        goal_raw[~valid]   = -1e9
        clear_raw[~valid]  = -1e9
        speed_raw[~valid]  = -1e9
        smooth_raw[~valid] = -1e9

        score = (self.k_goal   * self.norm(goal_raw,   valid) +
                 self.k_clear  * self.norm(clear_raw,  valid) +
                 self.k_speed  * self.norm(speed_raw,  valid) +
                 self.k_smooth * self.norm(smooth_raw, valid))
        score[~valid] = -1e9

        best = int(np.argmax(score))
        self.publish(float(vs[best]), float(ws[best]))

    def publish(self, v_des, w_des):
        """
        Aplica rampa de salida antes de publicar.
        Limita el delta de velocidad por ciclo para no saturar motores.
        """
        dv = clamp(v_des - self.sent_v, -self.ramp_v, self.ramp_v)
        dw = clamp(w_des - self.sent_w, -self.ramp_w, self.ramp_w)

        v_out = self.sent_v + dv
        w_out = self.sent_w + dw

        # si el destino es detenerse, permite bajar mas rapido (freno)
        if v_des == 0.0 and w_des == 0.0:
            v_out = clamp(self.sent_v - self.ramp_v * 2, 0.0, self.sent_v)
            w_out = clamp(self.sent_w - math.copysign(self.ramp_w * 2,
                                                       self.sent_w),
                          -abs(self.sent_w), abs(self.sent_w))
            w_out = 0.0 if abs(w_out) < 0.01 else w_out

        self.sent_v = v_out
        self.sent_w = w_out

        cmd = Twist()
        cmd.linear.x  = v_out
        cmd.angular.z = w_out
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = DWALocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()