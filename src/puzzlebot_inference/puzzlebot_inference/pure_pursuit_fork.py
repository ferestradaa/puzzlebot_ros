import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import tf2_ros

def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

def angle_wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.declare_parameter('v_max', 0.1)
        self.declare_parameter('w_max', 0.06)
        self.declare_parameter('a_lin', 0.10)
        self.declare_parameter('a_ang', 1.20)
        self.declare_parameter('ld_min', 0.25)
        self.declare_parameter('ld_max', 0.60)
        self.declare_parameter('ld_gain', 0.8)
        self.declare_parameter('goal_tol', 0.10)
        self.declare_parameter('pivot_angle', 1.2)
        self.declare_parameter('pivot_gain', 1.0)
        self.declare_parameter('align_band', 0.05)
        self.declare_parameter('final_yaw_tol', 0.05)
        self.declare_parameter('final_yaw_gain', 0.8)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('rate', 30.0)

        self.v_max          = self.get_parameter('v_max').value
        self.w_max          = self.get_parameter('w_max').value
        self.a_lin          = self.get_parameter('a_lin').value
        self.a_ang          = self.get_parameter('a_ang').value
        self.ld_min         = self.get_parameter('ld_min').value
        self.ld_max         = self.get_parameter('ld_max').value
        self.ld_gain        = self.get_parameter('ld_gain').value
        self.goal_tol       = self.get_parameter('goal_tol').value
        self.pivot_angle    = self.get_parameter('pivot_angle').value
        self.pivot_gain     = self.get_parameter('pivot_gain').value
        self.align_band     = self.get_parameter('align_band').value
        self.final_yaw_tol  = self.get_parameter('final_yaw_tol').value
        self.final_yaw_gain = self.get_parameter('final_yaw_gain').value
        self.map_frame      = self.get_parameter('map_frame').value
        rate = self.get_parameter('rate').value
        self.dt = 1.0 / rate

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # pose del fork_tip_link en frame mapa, actualizada via TF
        self.pose = None

        self.path      = []
        self.goal_yaw  = None
        self.aligning  = False
        self.prev_v    = 0.0
        self.prev_w    = 0.0

        self.create_subscription(Path, '/path', self.path_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_desired', 10)
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('pure pursuit ready, controlling fork_tip_link')

    def update_pose_from_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                'fork_tip_link',
                rclpy.time.Time()   # ultima transformacion disponible
            )
            tr = t.transform.translation
            self.pose = (tr.x, tr.y, yaw_from_quat(t.transform.rotation))
        except tf2_ros.LookupException:
            pass  # TF todavia no disponible
        except tf2_ros.ExtrapolationException:
            pass  # timestamp fuera de rango

    def path_cb(self, msg):
        self.path     = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.aligning = False
        if msg.poses:
            last_q = msg.poses[-1].pose.orientation
            if abs(last_q.w - 1.0) > 1e-3 or abs(last_q.z) > 1e-3:
                self.goal_yaw = yaw_from_quat(last_q)
                self.get_logger().info('goal yaw set to %.3f rad' % self.goal_yaw)
            else:
                self.goal_yaw = None

    def stop(self):
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.cmd_pub.publish(Twist())

    def ramp(self, target, prev, accel):
        step = accel * self.dt
        return prev + clamp(target - prev, -step, step)

    def publish_cmd(self, v_target, w_target):
        v = self.ramp(v_target, self.prev_v, self.a_lin)
        w = self.ramp(w_target, self.prev_w, self.a_ang)
        self.prev_v, self.prev_w = v, w
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def nearest_index(self, x, y):
        best_i, best_d = 0, float('inf')
        for i, (px, py) in enumerate(self.path):
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def lookahead_point(self, x, y, start_i, ld):
        for i in range(start_i, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - x, py - y) >= ld:
                return px, py
        return self.path[-1]

    def control_loop(self):
        self.update_pose_from_tf()

        if self.pose is None or len(self.path) < 2:
            self.stop()
            return

        x, y, th = self.pose
        gx, gy = self.path[-1]
        dist_to_goal = math.hypot(gx - x, gy - y)

        if self.aligning:
            if self.goal_yaw is None:
                self.stop()
                self.path    = []
                self.aligning = False
                self.get_logger().info('goal reached')
                return

            yaw_err = angle_wrap(self.goal_yaw - th)
            self.get_logger().info(
                'aligning: yaw_err=%.3f' % yaw_err,
                throttle_duration_sec=0.3)

            if abs(yaw_err) < self.final_yaw_tol:
                self.stop()
                self.path    = []
                self.aligning = False
                self.get_logger().info('goal reached with final orientation')
                return

            w_target = clamp(self.final_yaw_gain * yaw_err, -self.w_max, self.w_max)
            self.publish_cmd(0.0, w_target)
            return

        if dist_to_goal < self.goal_tol:
            self.stop()
            if self.goal_yaw is not None:
                self.aligning = True
                self.get_logger().info('position reached, starting final alignment')
            else:
                self.path = []
                self.get_logger().info('goal reached')
            return

        ld     = clamp(self.ld_gain * self.v_max, self.ld_min, self.ld_max)
        near_i = self.nearest_index(x, y)
        lx, ly = self.lookahead_point(x, y, near_i, ld)

        dx, dy = lx - x, ly - y
        x_r =  math.cos(th) * dx + math.sin(th) * dy
        y_r = -math.sin(th) * dx + math.cos(th) * dy
        alpha = math.atan2(y_r, x_r)
        dist  = math.hypot(x_r, y_r)

        self.get_logger().info(
            'th=%.2f alpha=%.2f look=(%.2f,%.2f) dist=%.2f' % (th, alpha, lx, ly, dist),
            throttle_duration_sec=0.5)

        if abs(alpha) < self.align_band:
            v_target = self.v_max
            w_target = 0.0
        elif abs(alpha) > self.pivot_angle:
            v_target = 0.0
            w_target = clamp(self.pivot_gain * alpha, -self.w_max, self.w_max)
        else:
            curvature = 2.0 * math.sin(alpha) / max(dist, 1e-3)
            v_target  = self.v_max * max(0.15, math.cos(alpha))
            w_target  = clamp(curvature * v_target, -self.w_max, self.w_max)

        self.publish_cmd(v_target, w_target)

def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()