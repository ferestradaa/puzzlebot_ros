#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from std_msgs.msg import Float32
from puzzlebot_interfaces.action import MoveForklift

import spidev
import struct
import time
import sys
from smbus2 import SMBus

INT32_MAX = 2_147_483_647
INT32_MIN = -2_147_483_648

CM_TO_FPGA_RAW = 2200.0 / 29.0
FPGA_INTERNAL_FACTOR = 1.26318


class ForkliftSPIServer(Node):
    def __init__(self):
        super().__init__('forklift_spi_server')

        try:
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)
            self.spi.max_speed_hz = 500_000
            self.spi.mode = 0b00
            self.spi.bits_per_word = 8
            self.spi.lsbfirst = False
        except Exception as e:
            self.get_logger().error(f'SPI open failed: {e}')
            sys.exit(1)

        self.current_target_raw = 0
        self.current_servo_angle = 90
        
        self.encoder_pulses = 0
        self.encoder_cm = 0.0
        self.encoder_offset_cm = 0.0
        
        self.calibrated = False
        self.homing_target_mm = 20.0
        self.homing_tolerance_mm = 3.0
        self.homing_step_down = -60
        self.homing_step_up = 60
        self.stable_count = 0

        self.laser_readings = []
        self.laser_distance_mm = 0.0

        self.laser_address = 0x29
        try:
            self.i2c_bus = SMBus(1)
            self.init_laser()
        except Exception as e:
            self.get_logger().error(f'I2C init failed: {e}')

        self.pub_height = self.create_publisher(Float32, '/forklift_height', 10)

        self.action_server = ActionServer(
            self,
            MoveForklift,
            'move_forklift',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.timer_homing = self.create_timer(0.1, self.homing_loop)
        self.timer_status = self.create_timer(0.1, self.publish_height)

        self.get_logger().info('Forklift SPI server started, running homing...')

    def init_laser(self):
        try:
            self.i2c_bus.write_byte_data(self.laser_address, 0x00, 0x00)
            time.sleep(0.01)
            self.i2c_bus.write_byte_data(self.laser_address, 0xFF, 0x01)
            self.i2c_bus.write_byte_data(self.laser_address, 0x00, 0x00)
            self.i2c_bus.write_byte_data(self.laser_address, 0x91, 0x3C)
            self.i2c_bus.write_byte_data(self.laser_address, 0x00, 0x01)
            self.i2c_bus.write_byte_data(self.laser_address, 0xFF, 0x00)
            self.i2c_bus.write_byte_data(self.laser_address, 0x80, 0x00)
            self.i2c_bus.write_byte_data(self.laser_address, 0x00, 0x02)
            time.sleep(0.01)
            return True
        except:
            return False

    def homing_loop(self):
        try:
            data = self.i2c_bus.read_i2c_block_data(self.laser_address, 0x1E, 2)
            raw_distance = float((data[0] << 8) + data[1])
            
            if raw_distance == 0.0:
                self.init_laser()
                self.send_spi(self.current_servo_angle, self.current_target_raw)
                return

            if raw_distance <= 10 or raw_distance > 2000:
                return

            self.laser_readings.append(raw_distance)
            if len(self.laser_readings) > 3:
                self.laser_readings.pop(0)
            
            self.laser_distance_mm = sum(self.laser_readings) / len(self.laser_readings)

            if self.calibrated:
                self.send_spi(self.current_servo_angle, self.current_target_raw)
                return

            error = self.laser_distance_mm - self.homing_target_mm

            if error > self.homing_tolerance_mm:
                new_raw = max(INT32_MIN, min(INT32_MAX, self.current_target_raw + self.homing_step_down))
                self.send_spi(self.current_servo_angle, new_raw)
                self.stable_count = 0
            elif error < -self.homing_tolerance_mm:
                new_raw = max(INT32_MIN, min(INT32_MAX, self.current_target_raw + self.homing_step_up))
                self.send_spi(self.current_servo_angle, new_raw)
                self.stable_count = 0
            else:
                self.stable_count += 1
                if self.stable_count >= 5:
                    self.calibrated = True
                    self.encoder_offset_cm = self.encoder_cm
                    self.send_spi(self.current_servo_angle, 0)
                    self.get_logger().info(f'Homing complete. Offset: {self.encoder_offset_cm:.2f} cm')

        except Exception as e:
            pass

    def goal_callback(self, goal_request):
        if not self.calibrated:
            self.get_logger().warn('Goal rejected: system not calibrated yet')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal canceled')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: {goal_handle.request.target_height_m:.3f} m')
        
        feedback_msg = MoveForklift.Feedback()
        target_cm = goal_handle.request.target_height_m * 100.0
        target_raw = int(round(target_cm * CM_TO_FPGA_RAW))
        
        self.send_spi(self.current_servo_angle, target_raw)
        
        rate = self.create_rate(10)
        tolerance_m = 0.01
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled by client')
                return MoveForklift.Result(success=False, final_height_m=self.get_absolute_height_m())
            
            current_height_m = self.get_absolute_height_m()
            error_m = abs(goal_handle.request.target_height_m - current_height_m)
            
            feedback_msg.current_height_m = current_height_m
            goal_handle.publish_feedback(feedback_msg)
            
            if error_m < tolerance_m:
                goal_handle.succeed()
                result = MoveForklift.Result()
                result.success = True
                result.final_height_m = current_height_m
                self.get_logger().info(f'Goal reached: {current_height_m:.3f} m')
                return result
            
            rate.sleep()

    def send_spi(self, servo_angle, target_raw):
        target_raw = max(INT32_MIN, min(INT32_MAX, target_raw))
        servo_angle = max(0, min(180, servo_angle))

        packet = struct.pack('>Bi', servo_angle, target_raw)
        tx_bytes = list(packet)

        try:
            rx_bytes = self.spi.xfer2(tx_bytes)
        except:
            return

        try:
            self.encoder_pulses = struct.unpack('>i', bytes(rx_bytes[1:5]))[0]
            
            target_pulses_fpga = target_raw * FPGA_INTERNAL_FACTOR
            if target_pulses_fpga != 0:
                self.encoder_cm = (self.encoder_pulses / target_pulses_fpga) * (target_raw / CM_TO_FPGA_RAW)
            else:
                self.encoder_cm = 0.0
        except:
            pass

        self.current_target_raw = target_raw
        self.current_servo_angle = servo_angle
        
        time.sleep(0.005)

    def get_absolute_height_m(self):
        return (self.encoder_cm - self.encoder_offset_cm) / 100.0

    def publish_height(self):
        if not self.calibrated:
            return
        
        msg = Float32()
        msg.data = self.get_absolute_height_m()
        self.pub_height.publish(msg)

    def destroy_node(self):
        self.spi.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftSPIServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()