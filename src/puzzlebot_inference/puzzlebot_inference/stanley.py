import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs 


class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        self.declare_parameter('k', 0.1)
        self.declare_parameter('v_ref', 0.08)
        self.declare_parameter('v_max', 0.08)
        self.declare_parameter('omega_max', 0.09)
        self.declare_parameter('goal_tol', 0.1)
        self.declare_parameter('alpha_pose', 0.04)    # suavizado de pose, más bajo = más suave
        self.declare_parameter('alpha_xy', 0.2)
        self.declare_parameter('alpha_yaw', 0.08)
        self.declare_parameter('cte_deadband', 0.03) # metros, ignorar error pequeño
        self.declare_parameter('alpha_steer', 0.35)  # suavizado de steering output

        self.k          = self.get_parameter('k').value
        self.v_ref      = self.get_parameter('v_ref').value
        self.v_max      = self.get_parameter('v_max').value
        self.omega_max  = self.get_parameter('omega_max').value
        self.goal_tol   = self.get_parameter('goal_tol').value
        self.alpha_pose  = self.get_parameter('alpha_pose').value
        self.cte_deadband = self.get_parameter('cte_deadband').value
        self.alpha_steer  = self.get_parameter('alpha_steer').value
        self.alpha_xy  = self.get_parameter('alpha_xy').value
        self.alpha_yaw = self.get_parameter('alpha_yaw').value

        self.x   = None
        self.y   = None
        self.yaw = None

        # steering suavizado
        self.smooth_omega = 0.0

        self.waypoints  = []
        self.current_wp = 0
        self.cmd        = Twist()

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_path = self.create_subscription(Path, '/path', self.path_cb, 10)
        self.pub_cmd  = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.1,  self.publish_cmd)

    def path_cb(self, msg):
        self.waypoints  = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_wp = 0
        self.get_logger().info(f'Path received: {len(self.waypoints)} waypoints')

    def odom_cb(self, msg):
        try:
            pose_in        = PoseStamped()
            pose_in.header = msg.header
            pose_in.pose   = msg.pose.pose
            pose_out = self.tf_buffer.transform(
                pose_in, 'map', timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except Exception as e:
            self.get_logger().warn(f'TF failed: {e}', throttle_duration_sec=1.0)
            return

        raw_x = pose_out.pose.position.x
        raw_y = pose_out.pose.position.y
        q = pose_out.pose.orientation
        raw_yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        if self.x is None:
            self.x, self.y, self.yaw = raw_x, raw_y, raw_yaw
            return

        self.x = self.alpha_xy * raw_x + (1.0 - self.alpha_xy) * self.x
        self.y = self.alpha_xy * raw_y + (1.0 - self.alpha_xy) * self.y

        dyaw = self._wrap(raw_yaw - self.yaw)
        self.yaw = self._wrap(self.yaw + self.alpha_yaw * dyaw)

    def control_loop(self):
        if self.x is None:
            return

        if not self.waypoints or self.current_wp >= len(self.waypoints):
            self.cmd = Twist()
            return

        tx, ty = self.waypoints[self.current_wp]
        dx = tx - self.x
        dy = ty - self.y
        dist = np.hypot(dx, dy)

        if dist < self.goal_tol:
            self.current_wp += 1
            if self.current_wp >= len(self.waypoints):
                self.get_logger().info('Goal reached')
                self.cmd      = Twist()
                self.smooth_omega = 0.0
            return

        path_yaw = np.arctan2(dy, dx)
        psi_e    = self._wrap(path_yaw - self.yaw)
        cte      = -np.sin(path_yaw) * (self.x - tx) + np.cos(path_yaw) * (self.y - ty)

        if abs(cte) < self.cte_deadband:
            cte = 0.0

        delta = psi_e + np.arctan2(self.k * cte, self.v_ref + 1e-5)

        v     = self.v_ref * (1.0 - min(abs(psi_e), np.pi / 2) / (np.pi / 2))
        v     = np.clip(v, 0.05, self.v_max)

        raw_omega = np.clip(delta, -self.omega_max, self.omega_max)

        self.smooth_omega = (
            self.alpha_steer * raw_omega
            + (1.0 - self.alpha_steer) * self.smooth_omega
        )

        self.cmd = Twist()
        self.cmd.linear.x  = v
        self.cmd.angular.z = self.smooth_omega

        self.get_logger().info(
            f'wp={self.current_wp} psi_e={np.degrees(psi_e):.1f} '
            f'cte={cte:.3f} v={v:.2f} w={self.smooth_omega:.2f}',
            throttle_duration_sec=0.2
        )

    def publish_cmd(self):
        self.pub_cmd.publish(self.cmd)

    def _wrap(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi


def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()