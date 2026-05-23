#!/usr/bin/env python3

import os
import math
import yaml
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped


class MapPoseVisualizer(Node):
    def __init__(self):
        super().__init__("map_pose_visualizer")

        self.declare_parameter("map_yaml", "map.yaml")
        self.declare_parameter("input_pose_topic", "/sim_ground_truth_pose")
        self.declare_parameter("output_pose_topic", "/robot_pose_map")
        self.declare_parameter("map_topic", "/visualization_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("slam_raw_pgm_order", True)

        self.map_yaml = self.get_parameter("map_yaml").value
        self.input_pose_topic = self.get_parameter("input_pose_topic").value
        self.output_pose_topic = self.get_parameter("output_pose_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.slam_raw_pgm_order = self.get_parameter("slam_raw_pgm_order").value

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)

        self.map_msg = self.load_map()

        self.pose_sub = self.create_subscription(
            Odometry,
            self.input_pose_topic,
            self.odom_callback,
            sensor_qos,
        )

        # If your /sim_ground_truth_pose is PoseStamped instead of Odometry,
        # comment the Odometry subscriber above and uncomment this:
        #
        # self.pose_sub = self.create_subscription(
        #     PoseStamped,
        #     self.input_pose_topic,
        #     self.pose_stamped_callback,
        #     sensor_qos,
        # )

        self.map_timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info(
            f"Publishing map on {self.map_topic} and robot pose on {self.output_pose_topic}"
        )

    def load_map(self):
        package_share = get_package_share_directory("puzzlebot_navigation")
        yaml_path = os.path.join(package_share, "maps", self.map_yaml)

        with open(yaml_path, "r") as f:
            info = yaml.safe_load(f)

        image_file = info["image"]
        resolution = float(info["resolution"])
        origin = info["origin"]

        image_path = os.path.join(os.path.dirname(yaml_path), image_file)
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        if img is None:
            raise RuntimeError(f"Could not load map image: {image_path}")

        height, width = img.shape

        occ = np.full((height, width), -1, dtype=np.int8)
        occ[img > 250] = 0
        occ[img <= 250] = 100

        if not self.slam_raw_pgm_order:
            occ = np.flipud(occ)

        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = occ.flatten().tolist()

        self.get_logger().info(
            f"Loaded map: {width}x{height}, res={resolution}, origin=({origin[0]}, {origin[1]}), "
            f"slam_raw_pgm_order={self.slam_raw_pgm_order}"
        )

        return msg

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

    def odom_callback(self, msg: Odometry):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.map_frame

        # map y turtlebot3_world son equivalentes, entonces copiamos pose directa.
        pose_msg.pose = msg.pose.pose

        self.pose_pub.publish(pose_msg)

    def pose_stamped_callback(self, msg: PoseStamped):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.map_frame

        # map y turtlebot3_world son equivalentes, entonces copiamos pose directa.
        pose_msg.pose = msg.pose

        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPoseVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()