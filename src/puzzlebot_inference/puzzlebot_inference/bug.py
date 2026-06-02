#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import numpy as np


class Bug2Navigator(Node):

    STATE_IDLE = 0
    STATE_GO_TO_GOAL = 1
    STATE_FOLLOW_WALL = 2
    STATE_DONE = 3

    def __init__(self):
        super().__init__('bug2_navigator')

        self.declare_parameter('linear_speed',       0.12)
        self.declare_parameter('angular_speed',      0.30)
        self.declare_parameter('wall_follow_dist',   0.35)
        self.declare_parameter('goal_tolerance',     0.1)
        self.declare_parameter('mline_tolerance',    0.12)
        self.declare_parameter('obstacle_threshold', 0.45)
        self.declare_parameter('max_linear_accel',   0.10)   # m/s^2
        self.declare_parameter('max_angular_accel',  0.50)   # rad/s^2

        self.linear_speed  = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.wall_dist     = self.get_parameter('wall_follow_dist').value
        self.goal_tol      = self.get_parameter('goal_tolerance').value
        self.mline_tol     = self.get_parameter('mline_tolerance').value
        self.obs_thresh    = self.get_parameter('obstacle_threshold').value
        self.max_lin_accel = self.get_parameter('max_linear_accel').value
        self.max_ang_accel = self.get_parameter('max_angular_accel').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        self.goal_x = None
        self.goal_y = None
        self.start_x = 0.0
        self.start_y = 0.0

        self.state = self.STATE_IDLE

        self.hit_x = None
        self.hit_y = None
        self.hit_dist_to_goal = None

        self.front_dist       = float('inf')
        self.front_left_dist  = float('inf')
        self.front_right_dist = float('inf')
        self.left_dist        = float('inf')
        self.right_dist       = float('inf')

        self.cmd_linear  = 0.0
        self.cmd_angular = 0.0
        self.dt = 0.05

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_cb, scan_qos)
        self.cmd_pub  = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, 10)

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('Bug2 ready. Waiting for goal on /goal_pose.')

    def goal_cb(self, msg):
        self.goal_x  = msg.pose.position.x
        self.goal_y  = msg.pose.position.y
        self.start_x = self.x
        self.start_y = self.y
        self.hit_x = None
        self.hit_y = None
        self.hit_dist_to_goal = None
        self.state = self.STATE_GO_TO_GOAL
        self.get_logger().info(f'New goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.odom_received = True

    def scan_cb(self, msg):
        ranges    = np.array(msg.ranges)
        ranges    = np.where(np.isfinite(ranges), ranges, float('inf'))
        n         = len(ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        def sector_min(center_rad, half_width_rad):
            c  = int((center_rad - angle_min) / angle_inc)
            hw = int(half_width_rad / angle_inc)
            indices = [(c + i) % n for i in range(-hw, hw + 1)]
            return float(np.min(ranges[indices]))

        half = math.radians(15)
        self.front_dist       = sector_min(0.0,               half)
        self.front_left_dist  = sector_min(math.radians( 45), half)
        self.front_right_dist = sector_min(math.radians(-45), half)
        self.left_dist        = sector_min(math.radians( 90), half)
        self.right_dist       = sector_min(math.radians(-90), half)

    def dist_to_goal(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def angle_to_goal(self):
        return math.atan2(self.goal_y - self.y, self.goal_x - self.x)

    def dist_to_mline(self):
        dx = self.goal_x - self.start_x
        dy = self.goal_y - self.start_y
        line_len = math.hypot(dx, dy)
        if line_len < 1e-6:
            return 0.0
        return abs(dy * self.x - dx * self.y + self.goal_x * self.start_y - self.goal_y * self.start_x) / line_len

    def on_mline(self):
        return self.dist_to_mline() < self.mline_tol

    def heading_toward_goal_on_mline(self):
        dx = self.goal_x - self.start_x
        dy = self.goal_y - self.start_y
        px = self.x - self.start_x
        py = self.y - self.start_y
        proj = (px * dx + py * dy) / (dx * dx + dy * dy + 1e-9)
        return proj > 0.05

    def publish_cmd(self, target_v, target_w):
        lin_step = self.max_lin_accel * self.dt
        ang_step = self.max_ang_accel * self.dt
        self.cmd_linear  += max(-lin_step, min(lin_step,  target_v - self.cmd_linear))
        self.cmd_angular += max(-ang_step, min(ang_step,  target_w - self.cmd_angular))
        msg = Twist()
        msg.linear.x  = self.cmd_linear
        msg.angular.z = self.cmd_angular
        self.cmd_pub.publish(msg)

    def go_to_goal_cmd(self):
        angle_err = self.angle_to_goal() - self.yaw
        angle_err = math.atan2(math.sin(angle_err), math.cos(angle_err))

        ang = max(-self.angular_speed, min(self.angular_speed, 1.5 * angle_err))

        # si el error de angulo es grande, gira en sitio primero
        # evita intentar avanzar a velocidades sub-umbral del motor
        if abs(angle_err) > math.radians(25):
            self.publish_cmd(0.0, ang)
            return

        # escala de heading: reduce velocidad lineal al alinearse
        angle_scale = 1.0 - min(abs(angle_err) / math.radians(25), 1.0) * 0.4

        # escala de scan: el techo real segun lo que hay enfrente
        clearance  = min(self.front_dist, self.front_left_dist, self.front_right_dist)
        scan_scale = min((clearance - self.obs_thresh) / self.obs_thresh, 1.0)
        scan_scale = max(scan_scale, 0.0)

        self.publish_cmd(self.linear_speed * min(angle_scale, scan_scale), ang)

    def follow_wall_cmd(self):
        err = self.left_dist - self.wall_dist

        if self.front_dist < self.obs_thresh:
            brake = min(self.front_dist / self.obs_thresh, 1.0)
            self.publish_cmd(
                self.linear_speed * 0.15 * brake,
                -self.angular_speed
            )
        elif self.front_left_dist < self.obs_thresh * 1.3:
            self.publish_cmd(self.linear_speed * 0.4, -self.angular_speed * 0.6)
        else:
            kp = 0.8
            ang_corr = max(-self.angular_speed * 0.7,
                           min(self.angular_speed * 0.7, kp * err))
            linear_scale = 1.0 - min(abs(ang_corr) / self.angular_speed, 1.0) * 0.4
            self.publish_cmd(self.linear_speed * linear_scale, ang_corr)

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def control_loop(self):
        if not self.odom_received:
            return

        if self.state == self.STATE_IDLE or self.state == self.STATE_DONE:
            self.stop()
            return

        dist = self.dist_to_goal()

        if dist < self.goal_tol:
            self.state = self.STATE_DONE
            self.stop()
            self.get_logger().info('Goal reached.')
            return

        if self.state == self.STATE_GO_TO_GOAL:
            if self.front_dist < self.obs_thresh:
                self.hit_x = self.x
                self.hit_y = self.y
                self.hit_dist_to_goal = dist
                self.state = self.STATE_FOLLOW_WALL
                self.get_logger().info(f'Hit obstacle at ({self.x:.2f}, {self.y:.2f})')
            else:
                self.go_to_goal_cmd()

        elif self.state == self.STATE_FOLLOW_WALL:
            if (self.on_mline()
                    and self.heading_toward_goal_on_mline()
                    and dist < self.hit_dist_to_goal - self.goal_tol
                    and self.front_dist >= self.obs_thresh):
                self.state = self.STATE_GO_TO_GOAL
                self.hit_x = None
                self.get_logger().info('Back on m-line, resuming go-to-goal.')
            else:
                self.follow_wall_cmd()


def main(args=None):
    rclpy.init(args=args)
    node = Bug2Navigator()
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