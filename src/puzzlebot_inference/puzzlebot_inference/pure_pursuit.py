import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist


def yaw_from_quat(q):
    # yaw only, planar robot
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # tunables. start gentle, the puzzlebot power budget is small
        self.declare_parameter('v_max', 0.08)        # m/s forward speed cap
        self.declare_parameter('w_max', 0.06)        # rad/s angular cap, NOT higher until brownout is gone
        self.declare_parameter('a_lin', 0.10)        # m/s^2 linear accel cap, limits current spike
        self.declare_parameter('a_ang', 1.20)        # rad/s^2 angular accel cap, limits current spike
        self.declare_parameter('ld_min', 0.25)       # m, lookahead lower bound
        self.declare_parameter('ld_max', 0.60)       # m, lookahead upper bound
        self.declare_parameter('ld_gain', 0.8)       # lookahead grows with speed: ld = gain * v
        self.declare_parameter('goal_tol', 0.10)     # m, distance to final point to stop
        self.declare_parameter('pivot_angle', 1.2)   # rad, if target this far off-axis, rotate in place
        self.declare_parameter('pivot_gain', 1.0)    # proportional gain when pivoting
        self.declare_parameter('align_band', 0.05)   # rad, deadband to stop micro jitter when aligned
        self.declare_parameter('rate', 30.0)         # Hz, control loop

        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.a_lin = self.get_parameter('a_lin').value
        self.a_ang = self.get_parameter('a_ang').value
        self.ld_min = self.get_parameter('ld_min').value
        self.ld_max = self.get_parameter('ld_max').value
        self.ld_gain = self.get_parameter('ld_gain').value
        self.goal_tol = self.get_parameter('goal_tol').value
        self.pivot_angle = self.get_parameter('pivot_angle').value
        self.pivot_gain = self.get_parameter('pivot_gain').value
        self.align_band = self.get_parameter('align_band').value
        rate = self.get_parameter('rate').value
        self.dt = 1.0 / rate

        self.pose = None          # (x, y, theta) in map frame
        self.path = []            # list of (x, y) in map frame
        self.prev_v = 0.0         # last commanded v, for accel limiting
        self.prev_w = 0.0         # last commanded w, for accel limiting

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Path, '/path', self.path_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('pure pursuit ready')

    def odom_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def path_cb(self, msg):
        # store path as plain xy list, assumed in the same frame as /odom (map)
        self.path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]

    def stop(self):
        # hard stop and reset the ramp so the next start ramps from zero
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.cmd_pub.publish(Twist())

    def ramp(self, target, prev, accel):
        # limit how fast the command can change, this is what stops the current spike
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
        # index of the closest path point to the robot
        best_i, best_d = 0, float('inf')
        for i, (px, py) in enumerate(self.path):
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def lookahead_point(self, x, y, start_i, ld):
        # walk forward from the nearest point until we pass the lookahead distance
        for i in range(start_i, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - x, py - y) >= ld:
                return px, py
        # none far enough, use the last point (we are near the goal)
        return self.path[-1]

    def control_loop(self):
        if self.pose is None or len(self.path) < 2:
            self.stop()
            return

        x, y, th = self.pose

        # goal check on the final waypoint
        gx, gy = self.path[-1]
        if math.hypot(gx - x, gy - y) < self.goal_tol:
            self.stop()
            self.path = []
            self.get_logger().info('goal reached')
            return

        # adaptive lookahead, grows with target speed
        ld = clamp(self.ld_gain * self.v_max, self.ld_min, self.ld_max)

        near_i = self.nearest_index(x, y)
        lx, ly = self.lookahead_point(x, y, near_i, ld)

        # lookahead point into robot frame
        dx, dy = lx - x, ly - y
        x_r = math.cos(th) * dx + math.sin(th) * dy
        y_r = -math.sin(th) * dx + math.cos(th) * dy

        alpha = math.atan2(y_r, x_r)   # angle to the lookahead point
        dist = math.hypot(x_r, y_r)    # actual lookahead distance

        # debug. if the robot only spins, watch alpha: if it never drops near 0 while
        # the robot is on the path, the heading (th) or the path frame is wrong.
        self.get_logger().info(
            'th=%.2f alpha=%.2f look=(%.2f,%.2f) dist=%.2f' % (th, alpha, lx, ly, dist),
            throttle_duration_sec=0.5)

        if abs(alpha) < self.align_band:
            # basically aligned, go straight, no angular jitter
            v_target = self.v_max
            w_target = 0.0
        elif abs(alpha) > self.pivot_angle:
            # target far off-axis, rotate in place. proportional so it eases in
            v_target = 0.0
            w_target = clamp(self.pivot_gain * alpha, -self.w_max, self.w_max)
        else:
            # pure pursuit curvature, scale speed down when off-axis
            curvature = 2.0 * math.sin(alpha) / max(dist, 1e-3)
            v_target = self.v_max * max(0.15, math.cos(alpha))
            w_target = clamp(curvature * v_target, -self.w_max, self.w_max)

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