#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

import spidev
import time
from smbus2 import SMBus

DIST_MIN_MM = 0.0
DIST_MAX_MM = 290.0

KP          = 0.8
DUTY_MIN    = 80
DUTY_MAX    = 95
PERIODO_PWM = 0.05
TOLERANCIA  = 3.0

ZONA_FRENADO_MM = 30.0
DUTY_FRENADO    = 80

DEVICE_ADDRESS = 0x29
OFFSET_MM      = 65.0

CMD_STOP = 0x00
CMD_SUBE = 0x01
CMD_BAJA = 0xFF

LIMITE_INFERIOR_MM = 10.0
LIMITE_SUPERIOR_MM = 280.0


class MastilNode(Node):

    def __init__(self):
        super().__init__('robot_spi_node')   # ← nombre del nodo cambiado

        try:
            self.bus = SMBus(1)
            self.limpiar_y_arrancar_laser()
            self.get_logger().info('Sensor VL53L0X iniciado')
        except Exception as e:
            self.get_logger().error(f'Error I2C: {e}')
            raise

        self.historial        = []
        self.altura_actual_mm = 0.0
        self.altura_previa_mm = 0.0

        try:
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)
            self.spi.max_speed_hz  = 100_000
            self.spi.mode          = 0
            self.spi.bits_per_word = 8
            self.spi.lsbfirst      = False
            self.get_logger().info('SPI iniciado')
        except Exception as e:
            self.get_logger().error(f'Error SPI: {e}')
            raise

        self.setpoint_mm      = None
        self.en_movimiento    = False
        self.direccion_previa = 0
        self.confirmaciones   = 0
        self.servo_angulo     = 90
        self.velocidad_manual = None

        self.pub_altura = self.create_publisher(
            Float32, 'forklift_height', 10)

        self.pub_servo = self.create_publisher(
            Int32, '/mastil/servo_angulo', 10)

        self.sub_setpoint = self.create_subscription(
            Float32, '/forklift/target_height',
            self.setpoint_callback, 10)

        self.sub_servo = self.create_subscription(
            Int32, '/mastil/servo_grados',
            self.servo_callback, 10)

        self.sub_velocidad = self.create_subscription(
            Int32, '/mastil/velocidad',
            self.velocidad_callback, 10)

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info(
            f'robot_spi_node listo (offset: {OFFSET_MM} mm)')



    def limpiar_y_arrancar_laser(self):
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
            self.get_logger().warn(f'Reinicio sensor falló: {e}')

    def leer_distancia(self):
        try:
            data  = self.bus.read_i2c_block_data(DEVICE_ADDRESS, 0x1E, 2)
            cruda = float((data[0] << 8) + data[1])
        except Exception as e:
            self.get_logger().warn(f'Error I2C: {e}')
            return None

        if cruda == 0.0:
            self.get_logger().warn('Lectura cero — reiniciando sensor...')
            self.limpiar_y_arrancar_laser()
            return None

        if cruda <= 10 or cruda > 2000:
            return None

        corregida = max(DIST_MIN_MM, cruda - OFFSET_MM)
        self.historial.append(corregida)
        if len(self.historial) > 3:
            self.historial.pop(0)

        return sum(self.historial) / len(self.historial)

    def _send(self, servo_ang, motor_cmd, duty=0):
        servo_ang = max(0, min(180, int(servo_ang)))
        duty      = max(0, min(95, int(duty)))
        self.spi.xfer([servo_ang, motor_cmd, duty], 100_000, 500)

    def motor_stop(self):
        self._send(self.servo_angulo, CMD_STOP, 0)

    def pulso_motor(self, direccion, duty):
        if not self.en_movimiento:
            self.motor_stop()
            return

        if self.velocidad_manual is not None:
            duty_int = self.velocidad_manual
        else:
            duty_int = max(DUTY_MIN, min(DUTY_MAX, int(duty)))

        cmd = CMD_SUBE if direccion == 1 else CMD_BAJA
        self._send(self.servo_angulo, cmd, duty_int)
        time.sleep(PERIODO_PWM)

    def direccion_real(self, distancia_actual):
        delta = distancia_actual - self.altura_previa_mm
        if   delta >  1.5:  return  1
        elif delta < -1.5:  return -1
        else:               return  0



    def setpoint_callback(self, msg: Float32):
        nuevo = max(DIST_MIN_MM, min(DIST_MAX_MM, float(msg.data)))
        self.setpoint_mm      = nuevo
        self.en_movimiento    = True
        self.direccion_previa = 0
        self.confirmaciones   = 0
        self.historial        = []
        self.get_logger().info(f'Nuevo setpoint: {nuevo:.1f} mm')

    def servo_callback(self, msg: Int32):
        angulo = max(0, min(180, int(msg.data)))
        self.servo_angulo = angulo
        self._send(self.servo_angulo, CMD_STOP, 0)
        pub_msg      = Int32()
        pub_msg.data = self.servo_angulo
        self.pub_servo.publish(pub_msg)
        self.get_logger().info(f'Servo → {angulo}°')

    def velocidad_callback(self, msg: Int32):
        vel = int(msg.data)
        if vel <= 0:
            self.velocidad_manual = None
            self.get_logger().info('Velocidad → AUTO (control P)')
        else:
            self.velocidad_manual = max(DUTY_MIN, min(DUTY_MAX, vel))
            self.get_logger().info(
                f'Velocidad → MANUAL {self.velocidad_manual}%')


    def loop(self):
        distancia = self.leer_distancia()

        if distancia is None:
            self.motor_stop()
            return

        if distancia >= LIMITE_SUPERIOR_MM:
            self.motor_stop()
            self.en_movimiento  = False
            self.setpoint_mm    = None
            self.confirmaciones = 0
            self.get_logger().warn(
                f'LÍMITE SUPERIOR: {distancia:.1f} mm — STOP')
            self.altura_previa_mm = distancia
            return

        self.altura_actual_mm = distancia

        msg      = Float32()
        msg.data = self.altura_actual_mm
        self.pub_altura.publish(msg)

        if self.setpoint_mm is None or not self.en_movimiento:
            self.motor_stop()
            self.altura_previa_mm = distancia
            return

        error             = self.setpoint_mm - self.altura_actual_mm
        direccion_deseada = 1 if error > 0 else -1
        dir_real          = self.direccion_real(distancia)

        if abs(error) <= TOLERANCIA:
            self.confirmaciones += 1
            self.motor_stop()

            if self.confirmaciones >= 3:
                sp = self.setpoint_mm
                dir_freno = -1 if error >= 0 else 1
                cmd_freno = CMD_SUBE if dir_freno == 1 else CMD_BAJA
                self._send(self.servo_angulo, cmd_freno, DUTY_MIN)
                time.sleep(0.03)
                self.motor_stop()
                self.motor_stop()
                self.motor_stop()
                self.en_movimiento  = False
                self.setpoint_mm    = None
                self.confirmaciones = 0
                self.get_logger().info(
                    f'✓ Objetivo confirmado 3x: {self.altura_actual_mm:.1f} mm '
                    f'(setpoint {sp:.1f} mm) — MOTOR STOP')
            else:
                self.get_logger().info(
                    f'[CONFIRMANDO {self.confirmaciones}/3] '
                    f'Altura: {distancia:.1f} mm | '
                    f'Error: {error:+.1f} mm')

            self.altura_previa_mm = distancia
            return

        self.confirmaciones = 0

        if dir_real == 1 and direccion_deseada == -1:
            cmd_final = 1
            escenario = 'ESC1↑'
        elif dir_real == -1 and direccion_deseada == 1:
            cmd_final = -1
            escenario = 'ESC2↓'
        elif dir_real == 1 and direccion_deseada == 1:
            cmd_final = 1
            escenario = 'ESC3✓↑'
        elif dir_real == -1 and direccion_deseada == -1:
            cmd_final = -1
            escenario = 'ESC4✓↓'
        else:
            cmd_final = direccion_deseada
            escenario = 'QUIETO'

        duty_p = KP * abs(error)

        if abs(error) < ZONA_FRENADO_MM:
            duty = max(DUTY_MIN, min(duty_p, DUTY_FRENADO))
        else:
            duty = max(DUTY_MIN, min(DUTY_MAX, duty_p))

        vel_tag = (f'MANUAL {self.velocidad_manual}%'
                   if self.velocidad_manual else f'AUTO {duty:.1f}%')

        self.get_logger().info(
            f'[{escenario}] Servo: {self.servo_angulo}° | '
            f'Altura: {distancia:.1f} mm | '
            f'Error: {error:+.1f} mm | '
            f'Vel: {vel_tag}')

        self.pulso_motor(cmd_final, duty)
        self.altura_previa_mm = distancia


    def destroy_node(self):
        self.motor_stop()
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
