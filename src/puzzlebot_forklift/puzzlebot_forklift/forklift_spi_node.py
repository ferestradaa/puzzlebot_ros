#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import spidev
import time
from smbus2 import SMBus

DIST_MIN_MM      = 0.0
DIST_MAX_MM      = 290.0
KP               = 0.8
DUTY_MIN         = 200
DUTY_MAX         = 250
PWM_PERIOD       = 0.05
TOLERANCE_MM     = 3.0
BRAKING_ZONE_MM  = 50.0
BRAKING_DUTY     = 200
DEVICE_ADDRESS   = 0x29
OFFSET_MM        = 55.0
CMD_STOP         = 0x00
CMD_UP           = 0x01
CMD_DOWN         = 0xFF
UPPER_LIMIT_MM   = 280.0


class MastilNode(Node):
    def __init__(self):
        super().__init__('forklift_spi_node')
        try:
            self.bus = SMBus(1)
            self._reset_sensor()
            self.get_logger().info('VL53L0X initialized')
        except Exception as e:
            self.get_logger().error(f'I2C error: {e}')
            raise

        self.history         = []
        self.current_height  = 0.0
        self.prev_height     = 0.0

        try:
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)
            self.spi.max_speed_hz  = 100_000
            self.spi.mode          = 0
            self.spi.bits_per_word = 8
            self.spi.lsbfirst      = False
            self.get_logger().info('SPI initialized')
        except Exception as e:
            self.get_logger().error(f'SPI error: {e}')
            raise

        self.setpoint_mm    = None
        self.moving         = False
        self.confirmations  = 0
        self.manual_speed   = None
        self.startup_cycles = 0

        self.pub_height = self.create_publisher(Float32, '/forklift/height', 10)
        self.sub_setpoint = self.create_subscription(
            Float32, '/forklift/setpoint', self.setpoint_callback, 10)

        self.timer = self.create_timer(0.05, self.loop)

    def _reset_sensor(self):
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
            self.get_logger().warn(f'Sensor reset failed: {e}')

    def _read_distance(self):
        try:
            data = self.bus.read_i2c_block_data(DEVICE_ADDRESS, 0x1E, 2)
            raw  = float((data[0] << 8) + data[1])
        except Exception as e:
            self.get_logger().warn(f'I2C read error: {e}')
            return None

        if raw == 0.0:
            self.get_logger().warn('Zero reading, resetting sensor')
            self._reset_sensor()
            return None

        if raw <= 10 or raw > 2000:
            return None

        corrected = max(0.0, raw - OFFSET_MM)
        self.history.append(corrected)
        if len(self.history) > 3:
            self.history.pop(0)
        return sum(self.history) / len(self.history)

    def _send(self, cmd, speed=255):
        self.spi.xfer([cmd, int(speed)], 100_000, 500)

    def _motor_stop(self):
        self._send(CMD_STOP, 0)

    def _motor_pulse(self, direction, duty):
        if not self.moving:
            self._motor_stop()
            return
        duty_int = int(max(DUTY_MIN, min(DUTY_MAX, duty)))
        cmd = CMD_UP if direction == 1 else CMD_DOWN
        self._send(cmd, duty_int)
        time.sleep(PWM_PERIOD)

    def _actual_direction(self, current_dist):
        delta = current_dist - self.prev_height
        if   delta >  1.5: return  1
        elif delta < -1.5: return -1
        else:              return  0

    def setpoint_callback(self, msg: Float32):
        target = max(DIST_MIN_MM, min(DIST_MAX_MM, float(msg.data)))
        self.setpoint_mm    = target
        self.moving         = True
        self.confirmations  = 0
        self.history        = []
        self.startup_cycles = 6
        self.get_logger().info(f'New setpoint: {target:.1f} mm')

    def speed_callback(self, msg: Int32):
        val = int(msg.data)
        if val <= 0:
            self.manual_speed = None
            self.get_logger().info('Speed -> AUTO')
        else:
            self.manual_speed = max(DUTY_MIN, min(DUTY_MAX, val))
            self.get_logger().info(f'Speed -> MANUAL {self.manual_speed}/255')

    def loop(self):
        dist = self._read_distance()
        if dist is None:
            self._motor_stop()
            return

        if dist >= UPPER_LIMIT_MM:
            self._motor_stop()
            self.moving        = False
            self.setpoint_mm   = None
            self.confirmations = 0
            self.get_logger().warn(f'Upper limit reached: {dist:.1f} mm')
            self.prev_height = dist
            return

        self.current_height = dist
        msg      = Float32()
        msg.data = self.current_height
        self.pub_height.publish(msg)

        if self.setpoint_mm is None or not self.moving:
            self._motor_stop()
            self.prev_height = dist
            return

        error        = self.setpoint_mm - self.current_height
        dir_desired  = 1 if error > 0 else -1
        dir_actual   = self._actual_direction(dist)

        if abs(error) <= TOLERANCE_MM:
            self.confirmations  += 1
            self.startup_cycles  = 0
            self._motor_stop()
            if self.confirmations >= 3:
                sp        = self.setpoint_mm
                brake_dir = -1 if error >= 0 else 1
                self._send(CMD_UP if brake_dir == 1 else CMD_DOWN, DUTY_MIN)
                time.sleep(0.03)
                self._motor_stop()
                self._motor_stop()
                self._motor_stop()
                self.moving        = False
                self.setpoint_mm   = None
                self.confirmations = 0
                self.get_logger().info(
                    f'Target reached 3x: {self.current_height:.1f} mm ({sp:.1f} mm)')
            else:
                self.get_logger().info(
                    f'[CONFIRMING {self.confirmations}/3] '
                    f'{dist:.1f} mm | err:{error:+.1f}')
            self.prev_height = dist
            return

        self.confirmations = 0

        if self.startup_cycles > 0:
            self.startup_cycles -= 1
            cmd_final = 1 if error > 0 else -1
            tag       = 'STARTUP'
            duty      = DUTY_MAX
        elif dir_desired == -1 and dir_actual == 1:
            cmd_final, tag = 1,  'INV_UP'
            duty = DUTY_MIN
        elif dir_desired == 1 and dir_actual == -1:
            cmd_final, tag = -1, 'INV_DOWN'
            duty = DUTY_MIN
        elif dir_desired == -1:
            cmd_final, tag = -1, 'DOWN'
            duty_p = KP * abs(error) * (255.0 / 100.0)
            if abs(error) < BRAKING_ZONE_MM:
                duty = max(DUTY_MIN, min(duty_p, BRAKING_DUTY))
            else:
                duty = max(DUTY_MIN, min(DUTY_MAX, duty_p))
        else:
            cmd_final, tag = 1, 'UP'
            duty_p = KP * abs(error) * (255.0 / 100.0)
            if abs(error) < BRAKING_ZONE_MM:
                duty = max(DUTY_MIN, min(duty_p, BRAKING_DUTY))
            else:
                duty = max(DUTY_MIN, min(DUTY_MAX, duty_p))

        speed_tag = (f'MANUAL {self.manual_speed}/255'
                     if self.manual_speed else f'AUTO {int(duty)}/255')
        self.get_logger().info(
            f'[{tag}] {dist:.1f}mm | err:{error:+.1f} | speed:{speed_tag}')

        self._motor_pulse(cmd_final, duty)
        self.prev_height = dist

    def destroy_node(self):
        self._motor_stop()
        self.spi.close()
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MastilNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()