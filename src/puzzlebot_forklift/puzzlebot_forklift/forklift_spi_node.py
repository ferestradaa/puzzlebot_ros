#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import time
from smbus2 import SMBus
import Jetson.GPIO as GPIO

PIN_IN1 = 29
PIN_IN2 = 31
PIN_ENA = 33

DIST_MIN          = 0.0
DIST_MAX          = 290.0
KP                = 0.8
DUTY_MIN          = 40.0
DUTY_MAX          = 95.0
PWM_PERIOD        = 0.05
TOLERANCE         = 3.0
BRAKING_ZONE      = 50.0
DUTY_BRAKING      = 50.0
DEVICE_ADDRESS    = 0x29
OFFSET            = 65.0
UPPER_LIMIT       = 280.0
ZERO_LIMIT        = 15.0
PWM_FREQUENCY     = 1000


class MastNode(Node):
    def __init__(self):
        super().__init__('robot_spi_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)

        try:
            self.bus = SMBus(1)
            self.reset_and_start_laser()
        except Exception as e:
            self.get_logger().error(f'I2C error: {e}')
            raise

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PIN_IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_IN2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_ENA, GPIO.OUT, initial=GPIO.LOW)
        self.pwm = GPIO.PWM(PIN_ENA, PWM_FREQUENCY)
        self.pwm.start(0)

        self.history        = []
        self.current_pos    = 0.0
        self.prev_pos       = 0.0
        self.setpoint       = None
        self.moving         = False
        self.confirmations  = 0
        self.manual_speed   = None
        self.startup_cycles = 0

        self._last_i2c_warn   = 0.0
        self._last_reset_warn = 0.0

        self.pub_height   = self.create_publisher(Float32, '/forklift/height', 10)
        self.sub_setpoint = self.create_subscription(
            Float32, '/forklift/setpoint', self.setpoint_callback, 10)
        self.sub_speed    = self.create_subscription(
            Int32, '/mast/speed', self.speed_callback, 10)
        self.timer = self.create_timer(0.05, self.loop)

    # SENSOR

    def reset_and_start_laser(self):
        try:
            self.bus.write_byte_data(DEVICE_ADDRESS, 0x00, 0x00)
            time.sleep(0.01)
            self.bus.write_byte_data(DEVICE_ADDRESS, 0xFF, 0x01)
            self.bus.write_byte_data(DEVICE_ADDRESS, 0x00, 0x00)
            self.bus.write_byte_data(DEVICE_ADDRESS, 0x91, 0x3C)
            self.bus.write_byte_data(DEVICE_ADDRESS, 0x00, 0x01)
            self.bus.write_byte_data(DEVICE_ADDRESS, 0xFF, 0x00)
            self.bus.write_byte_data(DEVICE_ADDRESS, 0x80, 0x00)
            self.bus.write_byte_data(DEVICE_ADDRESS, 0x00, 0x02)
            time.sleep(0.01)
        except Exception as e:
            now = time.time()
            if now - self._last_reset_warn > 5.0:
                self.get_logger().warn(f'Sensor reset failed: {e}')
                self._last_reset_warn = now

    def read_distance(self):
        try:
            data = self.bus.read_i2c_block_data(DEVICE_ADDRESS, 0x1E, 2)
            raw  = float((data[0] << 8) + data[1])
        except Exception as e:
            now = time.time()
            if now - self._last_i2c_warn > 5.0:
                self.get_logger().warn(f'I2C read error: {e}')
                self._last_i2c_warn = now
            return None

        if raw == 0.0:
            self.reset_and_start_laser()
            return None
        if raw <= 10 or raw >= 8190:
            return None

        corrected = max(0.0, raw - OFFSET)
        self.history.append(corrected)
        if len(self.history) > 5:
            self.history.pop(0)
        if len(self.history) < 3:
            return None

        sorted_h = sorted(self.history)
        median   = sorted_h[len(sorted_h) // 2]
        valid    = [x for x in self.history if abs(x - median) < 15.0]
        if not valid:
            return None

        avg = sum(valid) / len(valid)
        if avg < ZERO_LIMIT:
            avg = 0.0
        return avg

    # MOTOR

    def motor_stop(self):
        GPIO.output(PIN_IN1, GPIO.LOW)
        GPIO.output(PIN_IN2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)

    def motor_up(self, duty):
        duty = max(DUTY_MIN, min(DUTY_MAX, duty))
        GPIO.output(PIN_IN1, GPIO.HIGH)
        GPIO.output(PIN_IN2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(duty)

    def motor_down(self, duty):
        duty = max(DUTY_MIN, min(DUTY_MAX, duty))
        GPIO.output(PIN_IN1, GPIO.LOW)
        GPIO.output(PIN_IN2, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(duty)

    # ACTUAL DIRECTION

    def actual_direction(self, current):
        delta = current - self.prev_pos
        if   delta >  1.5: return  1
        elif delta < -1.5: return -1
        else:              return  0

    # CALLBACKS

    def setpoint_callback(self, msg: Float32):
        new = max(DIST_MIN, min(DIST_MAX, float(msg.data)))
        self.setpoint       = new
        self.moving         = True
        self.confirmations  = 0
        self.history        = []
        self.startup_cycles = 6

    def speed_callback(self, msg: Int32):
        vel = int(msg.data)
        if vel <= 0:
            self.manual_speed = None
        else:
            self.manual_speed = max(DUTY_MIN, min(DUTY_MAX, float(vel)))

    # MAIN LOOP

    def loop(self):
        distance = self.read_distance()
        if distance is None:
            self.motor_stop()
            return

        if distance >= UPPER_LIMIT:
            self.motor_stop()
            self.moving        = False
            self.setpoint      = None
            self.confirmations = 0
            self.get_logger().warn(f'UPPER LIMIT reached: {distance:.1f}')
            self.prev_pos = distance
            return

        self.current_pos = distance
        msg      = Float32()
        msg.data = self.current_pos
        self.pub_height.publish(msg)

        if self.setpoint is None or not self.moving:
            self.motor_stop()
            self.prev_pos = distance
            return

        error       = self.setpoint - self.current_pos
        desired_dir = 1 if error > 0 else -1
        actual_dir  = self.actual_direction(distance)

        # Dead zone - 3 confirmations
        if abs(error) <= TOLERANCE:
            self.confirmations   += 1
            self.startup_cycles   = 0
            self.motor_stop()
            if self.confirmations >= 3:
                if error >= 0:
                    self.motor_down(DUTY_MIN)
                else:
                    self.motor_up(DUTY_MIN)
                time.sleep(0.03)
                self.motor_stop()
                self.motor_stop()
                self.motor_stop()
                self.moving        = False
                self.setpoint      = None
                self.confirmations = 0
            self.prev_pos = distance
            return

        self.confirmations = 0

        # Startup grace cycles
        if self.startup_cycles > 0:
            self.startup_cycles -= 1
            duty = DUTY_MAX
            if error > 0:
                self.motor_up(duty)
            else:
                self.motor_down(duty)

        # Direction correction
        elif desired_dir == -1 and actual_dir == 1:
            self.motor_up(DUTY_MIN)
        elif desired_dir == 1 and actual_dir == -1:
            self.motor_down(DUTY_MIN)
        elif desired_dir == -1:
            duty_p = KP * abs(error)
            duty   = max(DUTY_MIN, min(duty_p, DUTY_BRAKING)) if abs(error) < BRAKING_ZONE else max(DUTY_MIN, min(DUTY_MAX, duty_p))
            self.motor_down(duty)
        else:
            duty_p = KP * abs(error)
            duty   = max(DUTY_MIN, min(duty_p, DUTY_BRAKING)) if abs(error) < BRAKING_ZONE else max(DUTY_MIN, min(DUTY_MAX, duty_p))
            self.motor_up(duty)

        time.sleep(PWM_PERIOD)
        self.prev_pos = distance

    # CLEANUP

    def destroy_node(self):
        self.motor_stop()
        self.pwm.stop()
        GPIO.cleanup()
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MastNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()