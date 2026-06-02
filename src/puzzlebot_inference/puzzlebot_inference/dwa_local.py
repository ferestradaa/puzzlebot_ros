import math
import numpy as np
import rclpy
from rclpy.node import Node
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

        # limites cinematicos
        self.declare_parameter('v_max', 0.25)
        self.declare_parameter('w_max', 0.2)

        # aceleraciones: subir agresivo, frenar agresivo
        self.declare_parameter('a_lin_up', 1.2)
        self.declare_parameter('a_lin_down', 2.0)
        self.declare_parameter('a_ang', 3.0)

        # muestreo de la ventana dinamica
        self.declare_parameter('v_samples', 10)
        self.declare_parameter('w_samples', 21)

        # horizonte de simulacion
        self.declare_parameter('sim_time', 2.0)
        self.declare_parameter('sim_dt', 0.1)

        # geometria y seguridad
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('clear_cap', 1.0)
        self.declare_parameter('scan_stride', 3)

        # seguimiento de path
        self.declare_parameter('lookahead', 0.7)
        self.declare_parameter('goal_tol', 0.10)

        # alineacion final
        self.declare_parameter('final_yaw_tol', 0.05)
        self.declare_parameter('final_yaw_gain', 0.8)

        # pesos — normalizados por ciclo, son escala-libre
        self.declare_parameter('k_goal', 1.0)
        self.declare_parameter('k_clear', 0.5)
        self.declare_parameter('k_speed', 0.6)
        self.declare_parameter('k_smooth', 0.2)

        # frames y rate
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('control_frame', 'base_link')
        self.declare_parameter('rate', 15.0)

        g = self.get_parameter
        self.v_max          = g('v_max').value
        self.w_max          = g('w_max').value
        self.a_lin_up       = g('a_lin_up').value
        self.a_lin_down     = g('a_lin_down').value
        self.a_ang          = g('a_ang').value
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
        self.control_frame  = g('control_frame').value

        rate = g('rate').value
        self.dt = 1.0 / rate
        self.n_steps = max(1, int(self.sim_time / self.sim_dt))

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path        = []
        self.goal_yaw    = None
        self.aligning    = False
        self.latest_scan = None
        self.cur_v       = 0.0
        self.cur_w       = 0.0

        self.create_subscription(Path,      '/path', self.path_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
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
        self.cur_v = msg.twist.twist.linear.x
        self.cur_w = msg.twist.twist.angular.z

    def get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.control_frame, rclpy.time.Time())
            tr = t.transform.translation
            return tr.x, tr.y, yaw_from_quat(t.transform.rotation)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return None

    def get_obstacles_map(self):
        scan = self.latest_scan
        if scan is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, scan.header.frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return None

        tx  = t.transform.translation.x
        ty  = t.transform.translation.y
        yaw = yaw_from_quat(t.transform.rotation)

        ranges = np.array(scan.ranges[::self.scan_stride], dtype=np.float64)
        n      = ranges.shape[0]
        angles = scan.angle_min + np.arange(n) * scan.angle_increment * self.scan_stride

        valid  = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        ranges = ranges[valid]
        angles = angles[valid]

        if ranges.shape[0] == 0:
            return np.empty((0, 2))

        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)
        mx = tx + math.cos(yaw) * lx - math.sin(yaw) * ly
        my = ty + math.sin(yaw) * lx + math.cos(yaw) * ly
        return np.stack([mx, my], axis=1)

    def nearest_index(self, x, y):
        best_i, best_d = 0, float('inf')
        for i, (px, py) in enumerate(self.path):
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def lookahead_target(self, x, y):
        near = self.nearest_index(x, y)
        for i in range(near, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - x, py - y) >= self.lookahead:
                return px, py
        return self.path[-1]

    def dynamic_window(self):
        v_lo = max(0.0,       self.cur_v - self.a_lin_down * self.dt)
        v_hi = min(self.v_max, self.cur_v + self.a_lin_up  * self.dt)
        w_lo = max(-self.w_max, self.cur_w - self.a_ang * self.dt)
        w_hi = min( self.w_max, self.cur_w + self.a_ang * self.dt)

        vs = np.linspace(v_lo, v_hi, self.v_samples)
        ws = np.linspace(w_lo, w_hi, self.w_samples)
        vv, ww = np.meshgrid(vs, ws)
        return vv.ravel(), ww.ravel()

    def rollout(self, x0, y0, th0, vs, ws):
        T   = self.n_steps
        ks  = np.arange(1, T + 1)
        dt  = self.sim_dt
        v   = vs[:, None]
        w   = ws[:, None]

        th       = th0 + w * ks * dt
        straight = np.abs(w) < 1e-4
        w_safe   = np.where(straight, 1.0, w)

        x_arc = x0 + (v / w_safe) * (np.sin(th) - math.sin(th0))
        y_arc = y0 - (v / w_safe) * (np.cos(th) - math.cos(th0))
        x_str = x0 + v * ks * dt * math.cos(th0)
        y_str = y0 + v * ks * dt * math.sin(th0)

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
        # normaliza solo sobre los candidatos validos para evitar que los -1e9 sesguen el rango
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

        x, y, th = pose
        gx, gy   = self.path[-1]
        dist_to_goal = math.hypot(gx - x, gy - y)

        if self.aligning:
            if self.goal_yaw is None:
                self.publish(0.0, 0.0)
                self.path     = []
                self.aligning = False
                return
            yaw_err = angle_wrap(self.goal_yaw - th)
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

        obs = self.get_obstacles_map()
        if obs is None:
            self.publish(0.0, 0.0)
            return

        tgt_x, tgt_y    = self.lookahead_target(x, y)
        vs, ws          = self.dynamic_window()
        xs, ys, ths     = self.rollout(x, y, th, vs, ws)
        clr             = self.clearances(xs, ys, obs)

        valid = clr >= self.robot_radius
        if not np.any(valid):
            bearing = angle_wrap(math.atan2(tgt_y - y, tgt_x - x) - th)
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
        smooth_raw = -np.abs(ws - self.cur_w)

        # invalida candidatos que colisionan antes de normalizar
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

    def publish(self, v, w):
        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = w
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