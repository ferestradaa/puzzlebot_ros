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
from std_msgs.msg import Bool, String  

RECOVERY_STATES = {'ESCAPE_BACKUP', 'ESCAPE_TURN', 'ESCAPE_FORWARD', 'STUCK_HOLD'}

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class ReactiveLayer(Node):
    def __init__(self):
        super().__init__('reactive_controller')

        self.declare_parameter('tip_offset', 0.2)
        self.declare_parameter('clearance', 0.15)
        self.declare_parameter('detect_dist', 0.6)
        self.declare_parameter('react_dist', 0.35)
        self.declare_parameter('cone_half_angle', 0.35)
        self.declare_parameter('evasion_gain', 0.6)
        self.declare_parameter('w_max', 0.3)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('corridor_half_width', 0.12)
        self.declare_parameter('corridor_min_x', 0.15)
        self.declare_parameter('escape_forward_speed', 0.12)
        self.declare_parameter('escape_forward_duration', 1.2)

        self.tip_offset = self.get_parameter('tip_offset').value
        self.clearance = self.get_parameter('clearance').value
        self.detect_dist = self.get_parameter('detect_dist').value
        self.react_dist = self.get_parameter('react_dist').value
        self.cone_half_angle = self.get_parameter('cone_half_angle').value
        self.evasion_gain = self.get_parameter('evasion_gain').value
        self.w_max = self.get_parameter('w_max').value
        self.base_frame = self.get_parameter('base_frame').value
        self.corridor_half_width = self.get_parameter('corridor_half_width').value
        self.corridor_min_x = self.get_parameter('corridor_min_x').value
        self.escape_forward_speed = self.get_parameter('escape_forward_speed').value
        self.escape_forward_duration = self.get_parameter('escape_forward_duration').value

        self.lidar_block_min = -0.0
        self.lidar_block_max = 0.0
        self.hard_stop = self.tip_offset + self.clearance

        self.a_lin_up = 0.15
        self.a_lin_down = 0.6
        self.a_ang = 0.8
        self.dt = 0.033
        self.dt_max = 0.1

        self.exit_margin = 0.2
        self.clear_needed = 5
        self.stuck_timeout = 0.8

        self.backup_speed = 0.1
        self.backup_duration = 1.0
        self.turn_timeout = 3.0
        self.max_escape_attempts = 2
        self.escape_turn_w = 0.25

        self.state = 'PASSTHROUGH'
        self.evade_dir = 0
        self.clear_frames = 0
        self.stuck_since = None
        self.phase_start = None
        self.escape_attempts = 0

        self.prev_v = 0.0
        self.prev_w = 0.0
        self._idle = False
        self._last_loop_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_scan = None
        self.latest_desired = None
        self.last_desired_time = None
        self.desired_timeout = Duration(seconds=0.5)
        self._loop_count = 0


        self._active_source = "pure_pursuit"
        self._bypass_sources = {"visual_servoing", "drive_raw"}
        self._in_bypass = False

        self._reverse = False

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_desired', self.desired_cb, 10)
        self.create_subscription(String, '/cmd_vel/active_source', self._source_cb, 10)
        self.create_subscription(Bool, '/pure_pursuit/reverse', self._reverse_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.stuck_pub = self.create_publisher(Bool, '/reactive_stuck', 10)
        self.create_timer(self.dt, self.control_loop)

    def _source_cb(self, msg):
        self._active_source = msg.data

    def scan_cb(self, msg):
        self.latest_scan = msg

    def desired_cb(self, msg):
        self.latest_desired = msg
        self.last_desired_time = self.get_clock().now()


    def _reverse_cb(self, msg: Bool):
        self._reverse = msg.data

    def get_laser_to_base_tf(self, frame_id):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, frame_id, rclpy.time.Time())
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            yaw = 2.0 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w)
            return tx, ty, yaw
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def analyze_cone(self, scan):
        tf = self.get_laser_to_base_tf(scan.header.frame_id)
        if tf is None:
            return float('inf'), 0, 0
        tx, ty, yaw = tf

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        n = ranges.shape[0]
        if n == 0:
            return float('inf'), 0, 0

        angles = scan.angle_min + np.arange(n, dtype=np.float32) * scan.angle_increment
        blocked = (angles >= self.lidar_block_min) & (angles <= self.lidar_block_max)
        ranges = ranges.copy()
        ranges[blocked] = np.inf

        valid = (np.isfinite(ranges) &
                 (ranges >= scan.range_min) &
                 (ranges <= scan.range_max))
        if not np.any(valid):
            return float('inf'), 0, 0

        r = ranges[valid]
        a = angles[valid]

        lx = r * np.cos(a)
        ly = r * np.sin(a)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        bx = tx + cos_y * lx - sin_y * ly
        by = ty + sin_y * lx + cos_y * ly



        if self._reverse:
            ahead = bx < -self.corridor_min_x
        else:
            ahead = bx > self.corridor_min_x

        bx = bx[ahead]
        by = by[ahead]
        if bx.size == 0:
            return float('inf'), 0, 0

        corridor = np.abs(by) <= self.corridor_half_width
        
        corridor_pts = bx[corridor]
        if corridor_pts.size >= 3:
            if self._reverse:
                forward_dist = float(-np.percentile(-corridor_pts, 10))
            else:
                forward_dist = float(np.percentile(corridor_pts, 10))
        else:
            forward_dist = float('inf')


        dist = np.hypot(bx, by)
        bearing = np.arctan2(by, bx)
        in_cone = (np.abs(bearing) <= self.cone_half_angle) & (dist < self.react_dist)
        left_count = int(np.sum(in_cone & (by >= 0.0)))
        right_count = int(np.sum(in_cone & (by < 0.0)))

        return forward_dist, left_count, right_count

    def pick_dir(self, left_count, right_count):
        return 1 if left_count <= right_count else -1

    def ramp(self, target, prev, a_up, a_down, dt):
        delta = target - prev
        decelerating = abs(target) < abs(prev)
        step = (a_down if decelerating else a_up) * dt
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
        self.state = 'ESCAPE_BACKUP'
        self.phase_start = now
        self.stuck_since = None

    def update_state(self, now, min_dist, left, right):
        laterals = left + right

        if self.state == 'PASSTHROUGH':
            if min_dist < self.detect_dist:
                self.state = 'AVOID'
                self.evade_dir = self.pick_dir(left, right)
                self.clear_frames = 0
                self.stuck_since = None

        elif self.state == 'AVOID':
            if min_dist > self.detect_dist + self.exit_margin and laterals == 0:
                self.clear_frames += 1
                if self.clear_frames >= self.clear_needed:
                    self.reset_to_passthrough()
            else:
                self.clear_frames = 0
            stall_threshold = self.hard_stop + 0.10
            if min_dist <= stall_threshold:
                if self.stuck_since is None:
                    self.stuck_since = now
                elif self.elapsed(self.stuck_since, now) > self.stuck_timeout:
                    self.enter_escape(now)
            else:
                self.stuck_since = None

        elif self.state == 'ESCAPE_BACKUP':
            if self.elapsed(self.phase_start, now) > self.backup_duration:
                self.state = 'ESCAPE_TURN'
                self.phase_start = now

        elif self.state == 'ESCAPE_TURN':
            if min_dist > self.detect_dist:
                self.state = 'ESCAPE_FORWARD'
                self.phase_start = now
            elif min_dist <= self.hard_stop:
                self.state = 'ESCAPE_BACKUP'
                self.phase_start = now
            elif self.elapsed(self.phase_start, now) > self.turn_timeout:
                self.escape_attempts += 1
                if self.escape_attempts >= self.max_escape_attempts:
                    self.state = 'STUCK_HOLD'
                else:
                    self.state = 'ESCAPE_BACKUP'
                    self.phase_start = now

        elif self.state == 'ESCAPE_FORWARD':
            if min_dist <= self.hard_stop:
                self.enter_escape(now)
            elif self.elapsed(self.phase_start, now) > self.escape_forward_duration:
                self.reset_to_passthrough()

        elif self.state == 'STUCK_HOLD':
            if min_dist > self.detect_dist:
                self.reset_to_passthrough()

    def command_for_state(self, min_dist, desired_v, desired_w):
        if self.state == 'PASSTHROUGH':
            return desired_v, desired_w

        if self.state == 'AVOID':
            if min_dist <= self.hard_stop:
                return 0.0, 0.0
            if min_dist >= self.react_dist:
                return desired_v, desired_w
            scale = clamp((min_dist - self.hard_stop) /
                          (self.react_dist - self.hard_stop), 0.0, 1.0)
            prox = clamp(1.0 - scale, 0.0, 1.0)
            v_target = desired_v * scale
            w_evasion = self.evade_dir * clamp(self.evasion_gain * prox, 0.0, self.w_max)
            w_target = prox * w_evasion + scale * desired_w
            return v_target, clamp(w_target, -self.w_max, self.w_max)

        if self.state == 'ESCAPE_BACKUP':
            speed = self.backup_speed if self._reverse else -self.backup_speed
            return speed, 0.0

        if self.state == 'ESCAPE_TURN':
            return 0.0, self.evade_dir * self.escape_turn_w

        if self.state == 'ESCAPE_FORWARD':
            speed = -self.escape_forward_speed if self._reverse else self.escape_forward_speed
            return speed, 0.0

        # STUCK_HOLD
        return 0.0, 0.0

    def publish_ramped(self, v_target, w_target, dt):
        cmd = Twist()
        cmd.linear.x = self.ramp(v_target, self.prev_v, self.a_lin_up, self.a_lin_down, dt)
        cmd.angular.z = self.ramp(w_target, self.prev_w, self.a_ang, self.a_ang, dt)
        self.prev_v = cmd.linear.x
        self.prev_w = cmd.angular.z
        self.cmd_pub.publish(cmd)
        return cmd.linear.x, cmd.angular.z

    def control_loop(self):
        now = self.get_clock().now()
        if self._last_loop_time is None:
            dt = self.dt
        else:
            dt = (now - self._last_loop_time).nanoseconds * 1e-9
            dt = clamp(dt, 0.0, self.dt_max)
        self._last_loop_time = now
        self._loop_count += 1
        log_this = (self._loop_count % 30 == 0)

        if self.latest_scan is None:
            if log_this:
                self.get_logger().warn('no scan yet')
            self.publish_ramped(0.0, 0.0, dt)
            return

        desired_ok = (self.latest_desired is not None and
                      self.last_desired_time is not None and
                      (now - self.last_desired_time).nanoseconds * 1e-9
                      <= self.desired_timeout.nanoseconds * 1e-9)
        if not desired_ok:
            if not self._idle:
                self.reset_to_passthrough()
                self._idle = True
            self.publish_ramped(0.0, 0.0, dt)
            self.stuck_pub.publish(Bool(data=False))
            return

        self._idle = False

        if self._active_source in self._bypass_sources:
            self.reset_to_passthrough()
            self.prev_v = 0.0
            self.prev_w = 0.0
            self._in_bypass = True
            self._last_loop_time = now
            return

        if self._in_bypass:
            self.prev_v = 0.0
            self.prev_w = 0.0
            self._in_bypass = False

        min_dist, left, right = self.analyze_cone(self.latest_scan)
        desired_v = self.latest_desired.linear.x
        desired_w = self.latest_desired.angular.z

        self.update_state(now, min_dist, left, right)
        v_target, w_target = self.command_for_state(min_dist, desired_v, desired_w)
        out_v, out_w = self.publish_ramped(v_target, w_target, dt)

        self.stuck_pub.publish(Bool(data=(self.state in RECOVERY_STATES)))


        '''
        if log_this:
            self.get_logger().info(
                f'{self.state} fwd={min_dist:.3f} dir={self.evade_dir} '
                f'des=({desired_v:.3f},{desired_w:.3f}) '
                f'cmd=({out_v:.3f},{out_w:.3f}) L={left} R={right}')
        '''
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