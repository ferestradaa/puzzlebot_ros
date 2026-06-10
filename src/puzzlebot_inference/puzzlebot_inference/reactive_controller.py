import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import tf2_ros
from tf2_ros import Buffer, TransformListener

# states that mean the reactive layer is recovering on its own
RECOVERY_STATES = {'ESCAPE_BACKUP', 'ESCAPE_TURN', 'STUCK_HOLD'}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ReactiveLayer(Node):
    def __init__(self):
        super().__init__('reactive_controller')

        # geometry and cone params
        self.declare_parameter('tip_offset', 0.2)
        self.declare_parameter('clearance', 0.15)
        self.declare_parameter('ramp_start', 0.8)
        self.declare_parameter('cone_half_angle', 0.524)
        self.declare_parameter('evasion_gain', 1.5)
        self.declare_parameter('w_max', 0.15)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('corridor_half_width', 0.25)  # half robot width + margin
        self.declare_parameter('corridor_min_x', 0.15)       # ignore points closer than this

        self.tip_offset = self.get_parameter('tip_offset').value
        self.clearance = self.get_parameter('clearance').value
        self.ramp_start = self.get_parameter('ramp_start').value
        self.cone_half_angle = self.get_parameter('cone_half_angle').value
        self.evasion_gain = self.get_parameter('evasion_gain').value
        self.w_max = self.get_parameter('w_max').value
        self.base_frame = self.get_parameter('base_frame').value
        self.corridor_half_width = self.get_parameter('corridor_half_width').value
        self.corridor_min_x = self.get_parameter('corridor_min_x').value

        # frontal stop distance, ramp zone reaches down to this
        self.hard_stop = self.tip_offset + self.clearance

        # accel limits for the command ramp, dt is the timer period
        self.dt = 0.033
        self.a_lin_up = 0.15    # gentle accel, protects the Jetson
        self.a_lin_down = 0.6   # aggressive braking, uses the ramp zone
        self.a_ang = 0.8

        # FSM tuning
        self.exit_margin = 0.2          # extra clearance over ramp_start to leave AVOID
        self.clear_needed = 5           # consecutive clean frames to leave AVOID
        self.stuck_timeout = 1.0        # seconds blocked at hard_stop before escaping
        self.backup_speed = 0.08        # reverse speed during escape
        self.backup_duration = 1.0      # reverse time, ~8cm of clearance
        self.turn_timeout = 3.0         # max in place rotation before retrying backup
        self.max_escape_attempts = 2    # after this, hold and hand off to the BT

        # FSM state
        self.state = 'PASSTHROUGH'      # PASSTHROUGH, AVOID, ESCAPE_BACKUP, ESCAPE_TURN, STUCK_HOLD
        self.evade_dir = 0              # +1 left, -1 right, latched
        self.clear_frames = 0
        self.stuck_since = None
        self.phase_start = None         # timer reference for escape phases
        self.escape_attempts = 0

        # command memory for the ramp
        self.prev_v = 0.0
        self.prev_w = 0.0

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # inputs
        self.latest_scan = None
        self.latest_desired = None
        self.last_desired_time = None
        self.desired_timeout = Duration(seconds=0.5)

        # diagnostics
        self._loop_count = 0
        self._last_scan_time = None

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_desired', self.desired_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.stuck_pub = self.create_publisher(Bool, '/reactive_stuck', 10)
        self.create_timer(self.dt, self.control_loop)

    def scan_cb(self, msg):
        now = self.get_clock().now()
        if self._last_scan_time is not None:
            dt = (now - self._last_scan_time).nanoseconds * 1e-9
            if dt > 0.15:
                self.get_logger().warn(f'scan gap {dt:.3f}s')
        self._last_scan_time = now
        self.latest_scan = msg

    def desired_cb(self, msg):
        self.latest_desired = msg
        self.last_desired_time = self.get_clock().now()

    def get_laser_to_base_tf(self, frame_id):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                frame_id,
                rclpy.time.Time()
            )
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            yaw = 2.0 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w)
            return tx, ty, yaw
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def analyze_cone(self, scan):
        # returns nearest frontal distance inside the robot corridor, and left/right
        # obstacle counts inside the wider steering cone. vectorized for the Jetson.
        tf = self.get_laser_to_base_tf(scan.header.frame_id)
        if tf is None:
            return float('inf'), 0, 0
        tx, ty, yaw = tf

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        n = ranges.shape[0]
        if n == 0:
            return float('inf'), 0, 0

        # correct per index angle, no off by one
        angles = scan.angle_min + np.arange(n, dtype=np.float32) * scan.angle_increment

        valid = (np.isfinite(ranges) &
                 (ranges >= scan.range_min) &
                 (ranges <= scan.range_max))
        if not np.any(valid):
            return float('inf'), 0, 0
        r = ranges[valid]
        a = angles[valid]

        # laser frame point to base frame
        lx = r * np.cos(a)
        ly = r * np.sin(a)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        bx = tx + cos_y * lx - sin_y * ly
        by = ty + sin_y * lx + cos_y * ly

        # drop points too close, likely self or sideways
        ahead = bx > self.corridor_min_x
        bx = bx[ahead]
        by = by[ahead]
        if bx.size == 0:
            return float('inf'), 0, 0

        # only points inside the robot corridor block forward motion
        corridor = np.abs(by) <= self.corridor_half_width
        forward_dist = float(np.min(bx[corridor])) if np.any(corridor) else float('inf')

        # lateral counts decide which way to turn, wider cone
        dist = np.hypot(bx, by)
        bearing = np.arctan2(by, bx)
        in_cone = (np.abs(bearing) <= self.cone_half_angle) & (dist < self.ramp_start)
        left_count = int(np.sum(in_cone & (by >= 0.0)))
        right_count = int(np.sum(in_cone & (by < 0.0)))

        return forward_dist, left_count, right_count

    def pick_dir(self, left_count, right_count):
        # turn toward the more open side, +1 left, -1 right
        return 1 if left_count <= right_count else -1

    def ramp(self, target, prev, a_up, a_down):
        delta = target - prev
        # reducing magnitude or crossing zero can be fast
        braking = (prev >= 0 and delta < 0) or (prev <= 0 and delta > 0)
        step = (a_down if braking else a_up) * self.dt
        return prev + clamp(delta, -step, step)

    def elapsed(self, t0, now):
        return (now - t0).nanoseconds * 1e-9

    def reset_to_passthrough(self):
        self.state = 'PASSTHROUGH'
        self.evade_dir = 0
        self.clear_frames = 0
        self.stuck_since = None
        self.escape_attempts = 0

    def enter_escape(self, now):
        # keep evade_dir latched, it already points to the more open side
        self.state = 'ESCAPE_BACKUP'
        self.phase_start = now
        self.stuck_since = None

    def update_state(self, now, min_dist, left, right):
        laterals = left + right

        if self.state == 'PASSTHROUGH':
            if min_dist < self.ramp_start:
                self.state = 'AVOID'
                self.evade_dir = self.pick_dir(left, right)  # latch once on entry
                self.clear_frames = 0
                self.stuck_since = None

        elif self.state == 'AVOID':
            # leave only with margin and several clean frames, schmitt trigger
            if min_dist > self.ramp_start + self.exit_margin and laterals == 0:
                self.clear_frames += 1
                if self.clear_frames >= self.clear_needed:
                    self.reset_to_passthrough()
            else:
                self.clear_frames = 0

            # stuck detection, blocked at the frontal stop
            if min_dist <= self.hard_stop:
                if self.stuck_since is None:
                    self.stuck_since = now
                elif self.elapsed(self.stuck_since, now) > self.stuck_timeout:
                    self.enter_escape(now)
            else:
                self.stuck_since = None

        elif self.state == 'ESCAPE_BACKUP':
            # blind reverse, time bounded, then rotate
            if self.elapsed(self.phase_start, now) > self.backup_duration:
                self.state = 'ESCAPE_TURN'
                self.phase_start = now

        elif self.state == 'ESCAPE_TURN':
            if min_dist > self.ramp_start:
                # front cleared, hand back to the global controller
                self.reset_to_passthrough()
            elif self.elapsed(self.phase_start, now) > self.turn_timeout:
                self.escape_attempts += 1
                if self.escape_attempts >= self.max_escape_attempts:
                    self.state = 'STUCK_HOLD'
                else:
                    self.state = 'ESCAPE_BACKUP'
                    self.phase_start = now

        elif self.state == 'STUCK_HOLD':
            # stopped, the BT must decide, recover only if the path opens by itself
            if min_dist > self.ramp_start:
                self.reset_to_passthrough()

    def command_for_state(self, min_dist, desired_v, desired_w):
        if self.state == 'PASSTHROUGH':
            return desired_v, desired_w

        if self.state == 'AVOID':
            if min_dist <= self.hard_stop:
                # too close, stop and let the stuck timer trigger the escape
                return 0.0, 0.0
            scale = clamp((min_dist - self.hard_stop) /
                          (self.ramp_start - self.hard_stop), 0.0, 1.0)
            v_target = desired_v * scale
            prox = clamp(1.0 - scale, 0.0, 1.0)
            # latched direction, not recomputed each frame, avoids sign flip
            evasion = self.evade_dir * clamp(self.evasion_gain * prox, 0.0, self.w_max)
            # blend, closer means evasion dominates, farther means desired_w dominates
            w_target = prox * evasion + (1.0 - prox) * desired_w
            return v_target, w_target

        if self.state == 'ESCAPE_BACKUP':
            # short blind reverse, no rear sensing, retracts the forks from the obstacle
            return -self.backup_speed, 0.0

        if self.state == 'ESCAPE_TURN':
            # rotate in place toward the latched open side
            return 0.0, self.evade_dir * self.w_max

        # STUCK_HOLD
        return 0.0, 0.0

    def control_loop(self):
        self._loop_count += 1
        log_this = (self._loop_count % 30 == 0)

        if self.latest_scan is None:
            if log_this:
                self.get_logger().warn('no scan received yet')
            return
        if self.latest_desired is None or self.last_desired_time is None:
            return

        now = self.get_clock().now()
        desired_age = (now - self.last_desired_time).nanoseconds * 1e-9
        if desired_age > self.desired_timeout.nanoseconds * 1e-9:
            # deadman, global controller is silent, full stop and reset
            self.prev_v = 0.0
            self.prev_w = 0.0
            self.reset_to_passthrough()
            self.cmd_pub.publish(Twist())
            self.stuck_pub.publish(Bool(data=False))
            return

        min_dist, left, right = self.analyze_cone(self.latest_scan)
        desired_v = self.latest_desired.linear.x
        desired_w = self.latest_desired.angular.z

        self.update_state(now, min_dist, left, right)
        v_target, w_target = self.command_for_state(min_dist, desired_v, desired_w)

        cmd = Twist()
        cmd.linear.x = self.ramp(v_target, self.prev_v, self.a_lin_up, self.a_lin_down)
        cmd.angular.z = self.ramp(w_target, self.prev_w, self.a_ang, self.a_ang)
        self.prev_v = cmd.linear.x
        self.prev_w = cmd.angular.z

        # tell the BT whether the reactive layer is recovering
        self.stuck_pub.publish(Bool(data=(self.state in RECOVERY_STATES)))

        if log_this:
            self.get_logger().info(
                f'{self.state} fwd={min_dist:.3f} dir={self.evade_dir} '
                f'des=({desired_v:.3f},{desired_w:.3f}) '
                f'cmd=({cmd.linear.x:.3f},{cmd.angular.z:.3f}) L={left} R={right}'
            )

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ReactiveLayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()