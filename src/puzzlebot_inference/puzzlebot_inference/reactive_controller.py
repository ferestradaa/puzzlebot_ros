import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class ReactiveLayer(Node):
    def __init__(self):
        super().__init__('reactive_layer')
        self.declare_parameter('tip_offset',      0.3)
        self.declare_parameter('clearance',       0.08)
        self.declare_parameter('ramp_start',      0.5)
        self.declare_parameter('cone_half_angle', 0.524)
        self.declare_parameter('evasion_gain',    1.5)
        self.declare_parameter('w_max',           0.18)
        self.declare_parameter('base_frame',      'base_link')

        self.tip_offset      = self.get_parameter('tip_offset').value
        self.clearance       = self.get_parameter('clearance').value
        self.ramp_start      = self.get_parameter('ramp_start').value
        self.cone_half_angle = self.get_parameter('cone_half_angle').value
        self.evasion_gain    = self.get_parameter('evasion_gain').value
        self.w_max           = self.get_parameter('w_max').value
        self.base_frame      = self.get_parameter('base_frame').value

        self.hard_stop = self.tip_offset + self.clearance

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_scan    = None
        self.latest_desired = None
        self.last_desired_time = None
        self.desired_timeout = Duration(seconds=0.5)

        self.prev_v = 0.0
        self.prev_w = 0.0
        self.a_lin  = 0.15
        self.a_ang  = 1.0

        # diagnostics
        self._loop_count      = 0
        self._last_scan_time  = None
        self._last_cmd_v      = 0.0
        self._last_cmd_w      = 0.0

        self.create_subscription(LaserScan, '/scan',           self.scan_cb,    10)
        self.create_subscription(Twist,     '/cmd_vel_desired', self.desired_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.033, self.control_loop)
        self.get_logger().info(
            f'reactive_layer init | hard_stop={self.hard_stop:.3f}m '
            f'ramp_start={self.ramp_start:.3f}m cone={math.degrees(self.cone_half_angle):.1f}deg'
        )

    def scan_cb(self, msg):
        now = self.get_clock().now()
        if self._last_scan_time is not None:
            dt = (now - self._last_scan_time).nanoseconds * 1e-9
            if dt > 0.15:
                self.get_logger().warn(f'scan gap {dt:.3f}s (expected ~0.1s)')
        self._last_scan_time = now
        self.latest_scan = msg

    def desired_cb(self, msg):
        self.get_logger().debug(
            f'desired_cb | v={msg.linear.x:.3f} w={msg.angular.z:.3f}'
        )
        self.latest_desired = msg
        self.last_desired_time = self.get_clock().now()

    def get_laser_to_base_tf(self, frame_id):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                frame_id,
                rclpy.time.Time()
            )
            tx  = tf.transform.translation.x
            ty  = tf.transform.translation.y
            yaw = 2.0 * math.atan2(
                tf.transform.rotation.z,
                tf.transform.rotation.w
            )
            return tx, ty, yaw
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'TF lookup failed ({frame_id}->{self.base_frame}): {e}', throttle_duration_sec=2.0)
            return None
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'TF extrapolation failed: {e}', throttle_duration_sec=2.0)
            return None

    def analyze_cone(self, scan):
        tf = self.get_laser_to_base_tf(scan.header.frame_id)
        if tf is None:
            self.get_logger().warn('analyze_cone: no TF, returning inf', throttle_duration_sec=2.0)
            return float('inf'), 0, 0

        tx, ty, yaw_offset = tf
        min_dist    = float('inf')
        left_count  = 0
        right_count = 0
        valid_pts   = 0
        angle       = scan.angle_min

        for r in scan.ranges:
            angle += scan.angle_increment
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue

            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            bx = tx + math.cos(yaw_offset) * lx - math.sin(yaw_offset) * ly
            by = ty + math.sin(yaw_offset) * lx + math.cos(yaw_offset) * ly
            bearing = math.atan2(by, bx)
            dist    = math.hypot(bx, by)

            if abs(bearing) <= self.cone_half_angle:
                valid_pts += 1
                if dist < min_dist:
                    min_dist = dist
                if dist < self.ramp_start:
                    if by >= 0.0:
                        left_count += 1
                    else:
                        right_count += 1

        self.get_logger().debug(
            f'cone | valid_pts={valid_pts} min_dist={min_dist:.3f}m '
            f'left={left_count} right={right_count}'
        )
        return min_dist, left_count, right_count

    def evasion_w(self, min_dist, left_count, right_count):
        proximity = 1.0 - (min_dist - self.hard_stop) / (self.ramp_start - self.hard_stop)
        proximity = clamp(proximity, 0.0, 1.0)
        magnitude = self.evasion_gain * proximity

        if left_count == 0 and right_count == 0:
            return 0.0
        if left_count <= right_count:
            w = clamp(magnitude, 0.0, self.w_max)
        else:
            w = clamp(-magnitude, -self.w_max, 0.0)

        self.get_logger().debug(
            f'evasion_w | prox={proximity:.2f} mag={magnitude:.3f} -> w={w:.3f}'
        )
        return w

    def ramp(self, target, prev, accel):
        step = accel * 0.033
        return prev + clamp(target - prev, -step, step)

    def control_loop(self):
        self._loop_count += 1
        log_this = (self._loop_count % 30 == 0)  # log a summary every ~1s

        if self.latest_scan is None:
            if log_this:
                self.get_logger().warn('control_loop: no scan received yet')
            return
        if self.latest_desired is None:
            if log_this:
                self.get_logger().warn('control_loop: no /cmd_vel_desired received yet')
            return
        if self.last_desired_time is None:
            return

        desired_age = (self.get_clock().now() - self.last_desired_time).nanoseconds * 1e-9
        if desired_age > self.desired_timeout.nanoseconds * 1e-9:
            if log_this:
                self.get_logger().warn(f'desired timeout ({desired_age:.2f}s) -> publishing zero')
            self.prev_v = 0.0
            self.prev_w = 0.0
            self.cmd_pub.publish(Twist())
            return

        min_dist, left_count, right_count = self.analyze_cone(self.latest_scan)

        if min_dist <= self.hard_stop:
            v_target = 0.0
            w_target = 0.0
            self.get_logger().warn(
                f'HARD STOP | dist={min_dist:.3f}m <= {self.hard_stop:.3f}m | '
                f'prev_v={self.prev_v:.3f} -> 0 | left={left_count} right={right_count}'
            )
            # freeze desired so upstream controller resumes cleanly
            self.latest_desired.linear.x  = 0.0
            self.latest_desired.angular.z = 0.0
        elif min_dist < self.ramp_start:
            scale    = (min_dist - self.hard_stop) / (self.ramp_start - self.hard_stop)
            scale    = clamp(scale, 0.0, 1.0)
            v_target = self.latest_desired.linear.x * scale
            w_target = self.evasion_w(min_dist, left_count, right_count)
            self.get_logger().info(
                f'RAMP ZONE | dist={min_dist:.3f}m scale={scale:.2f} '
                f'v_target={v_target:.3f} w_target={w_target:.3f}'
            )
        else:
            v_target = self.latest_desired.linear.x
            w_target = self.latest_desired.angular.z
            if log_this:
                self.get_logger().debug(
                    f'FREE | dist={min_dist:.3f}m v={v_target:.3f} w={w_target:.3f}'
                )

        cmd = Twist()
        cmd.linear.x  = self.ramp(v_target, self.prev_v, self.a_lin)
        cmd.angular.z = self.ramp(w_target, self.prev_w, self.a_ang)

        # log large velocity jumps (potential overcurrent trigger)
        dv = abs(cmd.linear.x - self._last_cmd_v)
        dw = abs(cmd.angular.z - self._last_cmd_w)
        if dv > 0.05 or dw > 0.15:
            self.get_logger().warn(
                f'VELOCITY JUMP | dv={dv:.3f} dw={dw:.3f} | '
                f'cmd v={cmd.linear.x:.3f} w={cmd.angular.z:.3f} | '
                f'prev v={self.prev_v:.3f} w={self.prev_w:.3f}'
            )

        self._last_cmd_v = cmd.linear.x
        self._last_cmd_w = cmd.angular.z
        self.prev_v      = cmd.linear.x
        self.prev_w      = cmd.angular.z
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