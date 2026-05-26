#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import spidev
import struct
import time
import sys
from smbus2 import SMBus

from puzzlebot_interfaces.srv import ForkliftHeight

INT32_MAX =  2_147_483_647
INT32_MIN = -2_147_483_648

CM_TO_FPGA_RAW = (2200.0 / 29.0) * 10
FPGA_INTERNAL_FACTOR = 1.26318 * 2.0


class ForkliftSPINode(Node):
    def __init__(self):
        super().__init__('forklift_spi_node')

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
        self.homing_target_mm = 25.0
        self.homing_tolerance_mm = 105.0
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

        self.srv_move = self.create_service(
            ForkliftHeight, 
            'forklift/target_height', 
            self.move_forklift_callback
        )

        self.pub_height = self.create_publisher(Float32, '/forklift/actual_height', 10)

        self.timer_main = self.create_timer(0.1, self.main_loop)

        self.get_logger().info('Forklift SPI node started, running homing...')

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
        
    def move_forklift_callback(self, request, response):
        if not self.calibrated:
            response.success = False
            response.message = "Not calibrated"
            return response
        
        target_cm = request.target_height * 100.0
        self.current_target_raw = int(round(target_cm * CM_TO_FPGA_RAW))
        response.success = True
        response.message = f"Target set to {request.target_height:.3f} m"
        return response

    def main_loop(self):
        try:
            data = self.i2c_bus.read_i2c_block_data(self.laser_address, 0x1E, 2)
            raw_distance = float((data[0] << 8) + data[1])
            
            if raw_distance == 0.0:
                self.init_laser()
                self.send_spi(self.current_servo_angle, self.current_target_raw)
                return

            if raw_distance <= 10 or raw_distance > 2000:
                self.send_spi(self.current_servo_angle, self.current_target_raw)
                self.publish_height()
                return

            self.laser_readings.append(raw_distance)
            if len(self.laser_readings) > 3:
                self.laser_readings.pop(0)
            
            self.laser_distance_mm = sum(self.laser_readings) / len(self.laser_readings)

            if not self.calibrated:
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
                        self.current_target_raw = 0
                        self.send_spi(self.current_servo_angle, 0)
                        self.get_logger().info(f'Homing complete. Offset: {self.encoder_offset_cm:.2f} cm')
            else:
                self.send_spi(self.current_servo_angle, self.current_target_raw)
                #self.get_logger().info(f'laser_raw={raw_distance:.1f}mm  target_raw={self.current_target_raw}')
                

            self.publish_height()

        except Exception as e:
            pass

    def send_spi(self, servo_angle, target_raw):
        target_raw = max(INT32_MIN, min(INT32_MAX, target_raw))
        servo_angle = max(0, min(180, servo_angle))
        #print(target_raw)
        #self.get_logger().info(f'TX: servo={servo_angle} raw={target_raw} | RX: {rx_bytes}')

        packet = struct.pack('>Bi', servo_angle, target_raw)
        tx_bytes = list(packet)

        try:
            rx_bytes = self.spi.xfer2(tx_bytes)
            #print(rx_bytes)
        except:
            return

        try:
            self.encoder_pulses = struct.unpack('>i', bytes(rx_bytes[1:5]))[0]
            self.encoder_cm = self.encoder_pulses / (CM_TO_FPGA_RAW * FPGA_INTERNAL_FACTOR)
            
        except:
            pass

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
    node = ForkliftSPINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()