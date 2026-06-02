import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import math


# parametros VFH
D_MAX = 0.7          # distancia maxima para considerar obstaculo
THRESHOLD = 2.0      # densidad del histograma para declarar sector bloqueado
SECTOR_SIZE_DEG = 5  # grados por sector
WAYPOINT_RADIUS = 0.15  # metros para considerar waypoint alcanzado
GOAL_RADIUS = 0.20   # metros para considerar goal final alcanzado

# parametros de velocidad
V_MAX = 0.15
W_MAX = 1.0
KW = 1.0             # ganancia proporcional para velocidad angular

NUM_SECTORS = 360 // SECTOR_SIZE_DEG


def build_polar_histogram(ranges, angle_min, angle_increment):
    histogram = np.zeros(NUM_SECTORS)
    sector_size_rad = np.radians(SECTOR_SIZE_DEG)

    for i, r in enumerate(ranges):
        if np.isinf(r) or np.isnan(r) or r <= 0.0:
            continue
        if r > D_MAX:
            continue

        angle = angle_min + i * angle_increment
        angle_pos = angle % (2 * np.pi)
        sector = int(angle_pos / sector_size_rad) % NUM_SECTORS

        magnitude = (D_MAX - r) / D_MAX
        histogram[sector] += magnitude

    return histogram


def find_best_sector(histogram, goal_sector):
    free = histogram < THRESHOLD

    if not np.any(free):
        return None

    best_sector = None
    best_diff = float('inf')

    for s in range(NUM_SECTORS):
        if not free[s]:
            continue
        diff = abs(s - goal_sector)
        diff = min(diff, NUM_SECTORS - diff)
        if diff < best_diff:
            best_diff = diff
            best_sector = s

    return best_sector


def sector_to_angle(sector):
    sector_size_rad = np.radians(SECTOR_SIZE_DEG)
    angle = sector * sector_size_rad
    if angle > np.pi:
        angle -= 2 * np.pi
    return angle


def compute_vfh(ranges, angle_min, angle_increment, goal_angle, goal_distance):
    if goal_distance < WAYPOINT_RADIUS:
        return 0.0, 0.0

    histogram = build_polar_histogram(ranges, angle_min, angle_increment)

    sector_size_rad = np.radians(SECTOR_SIZE_DEG)
    goal_angle_pos = goal_angle % (2 * np.pi)
    goal_sector = int(goal_angle_pos / sector_size_rad) % NUM_SECTORS

    best_sector = find_best_sector(histogram, goal_sector)

    if best_sector is None:
        return 0.0, W_MAX * 0.3

    best_angle = sector_to_angle(best_sector)

    angle_error = best_angle
    if angle_error > np.pi:
        angle_error -= 2 * np.pi
    elif angle_error < -np.pi:
        angle_error += 2 * np.pi

    w = float(np.clip(KW * angle_error, -W_MAX, W_MAX))

    valid_ranges = [r for r in ranges if not np.isinf(r) and not np.isnan(r) and r > 0.0]
    min_front_dist = min(valid_ranges) if valid_ranges else D_MAX

    dist_factor = float(np.clip(min_front_dist / D_MAX, 0.0, 1.0))
    angle_factor = 1.0 - float(np.clip(abs(angle_error) / np.pi, 0.0, 1.0))

    v = V_MAX * dist_factor * angle_factor

    return v, w


class VFHNode(Node):
    def __init__(self):
        super().__init__('vfh_node')

        # estado
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.waypoints = []       # lista de (x, y)
        self.waypoint_idx = 0
        self.scan = None
        self.goal_reached = False

        # suscriptores
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/odom', self.pose_callback, 10)

        self.path_sub = self.create_subscription(
            Path, '/path', self.path_callback, 10)

        # publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # timer de control a 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('VFH node started')

    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def path_callback(self, msg: Path):
        self.waypoints = []
        self.waypoint_idx = 0
        self.goal_reached = False

        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.waypoints.append((x, y))

        self.get_logger().info(f'Path received: {len(self.waypoints)} waypoints')

    def get_goal_polar(self):
        # retorna (goal_angle en frame robot, goal_distance) hacia waypoint activo
        if not self.waypoints or self.waypoint_idx >= len(self.waypoints):
            return 0.0, 0.0

        wx, wy = self.waypoints[self.waypoint_idx]

        dx = wx - self.robot_x
        dy = wy - self.robot_y

        distance = math.sqrt(dx**2 + dy**2)
        angle_world = math.atan2(dy, dx)

        # angulo relativo al heading del robot
        goal_angle = angle_world - self.robot_yaw
        goal_angle = math.atan2(math.sin(goal_angle), math.cos(goal_angle))

        return goal_angle, distance

    def advance_waypoint(self):
        if self.waypoint_idx < len(self.waypoints) - 1:
            self.waypoint_idx += 1
            self.get_logger().info(
                f'Waypoint {self.waypoint_idx}/{len(self.waypoints) - 1}')
        else:
            self.goal_reached = True
            self.get_logger().info('Goal reached')

    def control_loop(self):
        if self.scan is None:
            return

        if not self.waypoints:
            return

        if self.goal_reached:
            self.publish_cmd(0.0, 0.0)
            return

        goal_angle, goal_distance = self.get_goal_polar()

        # verifica si llego al waypoint activo
        if goal_distance < WAYPOINT_RADIUS:
            self.advance_waypoint()
            return

        # verifica si el waypoint activo es el ultimo y esta dentro del goal radius
        is_last = (self.waypoint_idx == len(self.waypoints) - 1)
        if is_last and goal_distance < GOAL_RADIUS:
            self.goal_reached = True
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('Goal reached')
            return

        v, w = compute_vfh(
            ranges=list(self.scan.ranges),
            angle_min=self.scan.angle_min,
            angle_increment=self.scan.angle_increment,
            goal_angle=goal_angle,
            goal_distance=goal_distance
        )

        self.publish_cmd(v, w)

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VFHNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()