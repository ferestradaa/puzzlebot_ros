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
        self.declare_parameter('tip_offset',      0.2)
        self.declare_parameter('clearance',       0.15)
        self.declare_parameter('ramp_start',      0.8)
        self.declare_parameter('cone_half_angle', 0.524)
        self.declare_parameter('evasion_gain',    1.5)
        self.declare_parameter('w_max',           0.15)
        self.declare_parameter('base_frame',      'base_link')
        self.declare_parameter('corridor_half_width', 0.25)  # medio ancho del robot + margen

        self.a_lin_up   = 0.15   # acelerar gentil, protege el Jetson
        self.a_lin_down = 0.6    # frenar agresivo, usa la zona de rampa
        self.a_ang      = 1.0
        self.corridor_half_width = self.get_parameter('corridor_half_width').value

        
        self.tip_offset      = self.get_parameter('tip_offset').value
        self.clearance       = self.get_parameter('clearance').value
        self.ramp_start      = self.get_parameter('ramp_start').value
        self.cone_half_angle = self.get_parameter('cone_half_angle').value
        self.evasion_gain    = self.get_parameter('evasion_gain').value
        self.w_max           = self.get_parameter('w_max').value
        self.base_frame      = self.get_parameter('base_frame').value


        self.declare_parameter('corridor_min_x', 0.15)  # ignora puntos mas cerca que esto (ruido/self/lado)
        self.corridor_min_x = self.get_parameter('corridor_min_x').value
        # hard_stop = 0.35m, ramp desde 0.8m -> 65cm de margen
        self.hard_stop = self.tip_offset + self.clearance
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_scan       = None
        self.latest_desired    = None
        self.last_desired_time = None
        self.desired_timeout   = Duration(seconds=0.5)
        self.prev_v = 0.0
        self.prev_w = 0.0
        self.a_lin  = 0.05
        self.a_ang  = 0.8
        self._loop_count     = 0
        self._last_scan_time = None
        self._last_cmd_v     = 0.0
        self._last_cmd_w     = 0.0
        self.create_subscription(LaserScan, '/scan',            self.scan_cb,    10)
        self.create_subscription(Twist,     '/cmd_vel_desired', self.desired_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.033, self.control_loop)



    def scan_cb(self, msg):
        now = self.get_clock().now()
        if self._last_scan_time is not None:
            dt = (now - self._last_scan_time).nanoseconds * 1e-9
            if dt > 0.15:
                self.get_logger().warn(f'scan gap {dt:.3f}s')
        self._last_scan_time = now
        self.latest_scan = msg
    def desired_cb(self, msg):
        self.latest_desired    = msg
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
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return None
        
    def analyze_cone(self, scan):
        tf = self.get_laser_to_base_tf(scan.header.frame_id)
        if tf is None:
            return float('inf'), 0, 0

        tx, ty, yaw_offset = tf
        forward_dist = float('inf')   # distancia frontal solo dentro del corredor del robot
        left_count   = 0
        right_count  = 0
        angle        = scan.angle_min

        for r in scan.ranges:
            angle += scan.angle_increment
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue

            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            bx = tx + math.cos(yaw_offset) * lx - math.sin(yaw_offset) * ly
            by = ty + math.sin(yaw_offset) * lx + math.cos(yaw_offset) * ly

            if bx <= self.corridor_min_x:
                continue  # demasiado cerca, probablemente al lado o auto-deteccion

            # frenado: solo puntos dentro del ancho del robot bloquean el avance
            # un punto de lado no frena, se esquiva con el giro
            if abs(by) <= self.corridor_half_width and bx < forward_dist:
                forward_dist = bx

            # conteo lateral para decidir hacia donde esquivar, cono mas amplio
            dist    = math.hypot(bx, by)
            bearing = math.atan2(by, bx)
            if abs(bearing) <= self.cone_half_angle and dist < self.ramp_start:
                if by >= 0.0:
                    left_count += 1
                else:
                    right_count += 1

        return forward_dist, left_count, right_count
    

    def evasion_w(self, min_dist, left_count, right_count, desired_w):
        proximity = 1.0 - (min_dist - self.hard_stop) / (self.ramp_start - self.hard_stop)
        proximity = clamp(proximity, 0.0, 1.0)
        magnitude = self.evasion_gain * proximity
        if left_count == 0 and right_count == 0:
            # no hay puntos laterales en la zona de peligro, pasa el giro deseado
            return desired_w
        if left_count <= right_count:
            evasion = clamp(magnitude, 0.0, self.w_max)
        else:
            evasion = clamp(-magnitude, -self.w_max, 0.0)
        # blend: a mayor proximidad, domina evasion; a menor, domina desired_w
        # esto evita el flip de signo que causaba la oscilacion en arco
        blend = clamp(proximity, 0.0, 1.0)
        return blend * evasion + (1.0 - blend) * desired_w
    

    def ramp(self, target, prev, a_up, a_down):
        delta = target - prev
        # frenar (reducir magnitud o acercarse a cero) puede ser rapido
        braking = (prev >= 0 and delta < 0) or (prev <= 0 and delta > 0)
        step = (a_down if braking else a_up) * 0.033
        return prev + clamp(delta, -step, step)
    

    def control_loop(self):
        self._loop_count += 1
        log_this = (self._loop_count % 30 == 0)
        if self.latest_scan is None:
            if log_this:
                self.get_logger().warn('no scan received yet')
            return
        if self.latest_desired is None or self.last_desired_time is None:
            return
        desired_age = (self.get_clock().now() - self.last_desired_time).nanoseconds * 1e-9
        if desired_age > self.desired_timeout.nanoseconds * 1e-9:
            self.prev_v = 0.0
            self.prev_w = 0.0
            self.cmd_pub.publish(Twist())
            return
        min_dist, left_count, right_count = self.analyze_cone(self.latest_scan)
        desired_v = self.latest_desired.linear.x
        desired_w = self.latest_desired.angular.z
        is_pivoting = abs(desired_v) < 0.05 and abs(desired_w) > 0.05 and min_dist > self.hard_stop
        if is_pivoting:
            # giro en lugar: no frenar por obstaculo frontal, rotar saca de la esquina
            v_target = desired_v
            w_target = desired_w
        elif min_dist <= self.hard_stop:
            v_target = 0.0
            w_target = 0.0

            self.latest_desired.linear.x  = 0.0
            self.latest_desired.angular.z = 0.0
        elif min_dist < self.ramp_start:
            scale    = (min_dist - self.hard_stop) / (self.ramp_start - self.hard_stop)
            scale    = clamp(scale, 0.0, 1.0)
            v_target = desired_v * scale
            w_target = self.evasion_w(min_dist, left_count, right_count, desired_w)
        else:
            v_target = desired_v
            w_target = desired_w
        # al final de control_loop, antes de publicar

        cmd = Twist()
# en control_loop al publicar
        cmd.linear.x  = self.ramp(v_target, self.prev_v, self.a_lin_up, self.a_lin_down)
        cmd.angular.z = self.ramp(w_target, self.prev_w, self.a_ang, self.a_ang)
        self._last_cmd_v = cmd.linear.x
        self._last_cmd_w = cmd.angular.z
        self.prev_v      = cmd.linear.x
        self.prev_w      = cmd.angular.z



        if log_this:
            branch = ('PIVOT' if is_pivoting else
                    'HARDSTOP' if min_dist <= self.hard_stop else
                    'RAMP' if min_dist < self.ramp_start else 'FREE')
            self.get_logger().info(
                f'{branch} | fwd={min_dist:.3f} desv=({desired_v:.3f},{desired_w:.3f}) '
                f'cmd=({cmd.linear.x:.3f},{cmd.angular.z:.3f}) L={left_count} R={right_count}'
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