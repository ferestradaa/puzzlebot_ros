# ros2/sim_bridge.py

import yaml
import threading
from geometry_msgs.msg import Twist
from rclpy.node import Node

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))  # sube a simulation/
from robot.drive_controller import DriveController


class SimBridge(Node):

    def __init__(self, config: dict):
        super().__init__("cmd_vel_bridge")

        self.config = config
        self._lock  = threading.Lock()

        self._linear_x  = 0.0
        self._angular_z = 0.0
        self.drive = None

        self.drive = DriveController(config)

        cmd_vel_topic = config["ros2"]["topics"]["cmd_vel"]
        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_cb, 10)

        self.get_logger().info(f"Listening to {cmd_vel_topic}")

    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._linear_x  = msg.linear.x
            self._angular_z = msg.angular.z

    def step(self, dt: float):

        """instead of seting it every ros2 calllback this is called by step thread (running in simulation main)"""
        with self._lock:
            linear_x  = self._linear_x
            angular_z = self._angular_z

        self.drive.set_twist(linear_x, angular_z)

    def ensure_initialized(self):
            if self.drive is None:
                self.drive = DriveController(self.config)  