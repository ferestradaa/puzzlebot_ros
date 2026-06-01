import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import numpy as np


from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        self.get_logger().info('Node starting')

        self.declare_parameter('k', 0.2)
        self.declare_parameter('v_ref', 0.08)
        self.declare_parameter('v_max', 0.08)
        self.declare_parameter('omega_max', 0.09)
        self.declare_parameter('goal_tol', 0.1)

        self.k = self.get_parameter('k').value
        self.v_ref = self.get_parameter('v_ref').value
        self.v_max = self.get_parameter('v_max').value
        self.omega_max = self.get_parameter('omega_max').value
        self.goal_tol = self.get_parameter('goal_tol').value

        self.waypoints = []
        self.current_wp = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.cmd = Twist()

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_path = self.create_subscription(Path, '/path', self.path_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # control loop separado del pub
        self.create_timer(0.05, self.control_loop)
        # publica a 10hz para no saturar micro-ros agent
        self.create_timer(0.1, self.publish_cmd)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def path_cb(self, msg):
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_wp = 0
        self.get_logger().info(f'Path received: {len(self.waypoints)} waypoints')

    # reemplaza odom_cb:
    def odom_cb(self, msg):
        try:
            pose_in = PoseStamped()
            pose_in.header = msg.header
            pose_in.pose = msg.pose.pose
            pose_out = self.tf_buffer.transform(pose_in, 'map', timeout=rclpy.duration.Duration(seconds=0.1))
            self.x = pose_out.pose.position.x
            self.y = pose_out.pose.position.y
            q = pose_out.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.yaw = np.arctan2(siny, cosy)
        except Exception as e:
            self.get_logger().warn(f'TF failed: {e}')

            
    def control_loop(self):
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
                self.cmd = Twist()
            return

        path_yaw = np.arctan2(dy, dx)
        psi_e = self._wrap(path_yaw - self.yaw)
        e = -np.sin(path_yaw) * (self.x - tx) + np.cos(path_yaw) * (self.y - ty)

        delta = psi_e + np.arctan2(self.k * e, self.v_ref + 1e-5)

        v = self.v_ref * (1.0 - min(abs(psi_e), np.pi/2) / (np.pi / 2))
        v = np.clip(v, 0.05, self.v_max)
        omega = np.clip(delta, -self.omega_max, self.omega_max)

        self.cmd = Twist()
        self.cmd.linear.x = v
        self.cmd.angular.z = omega

        self.get_logger().info(
            f'wp={self.current_wp} psi_e={np.degrees(psi_e):.1f} e={e:.2f} v={v:.2f} w={omega:.2f}'
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