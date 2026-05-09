#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        self.declare_parameter('interval', 0.5)
        self.declare_parameter('output_dir', os.path.expanduser('~/saved_frames'))

        self.interval = self.get_parameter('interval').value
        self.output_dir = self.get_parameter('output_dir').value

        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.last_save = 0.0

        self.sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info(f'Saving frames every {self.interval}s to {self.output_dir}')

    def image_callback(self, msg):
        now = time.time()
        if now - self.last_save < self.interval:
            return

        self.last_save = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        timestamp = int(now * 1000)
        path = os.path.join(self.output_dir, f'frame_{timestamp}.jpg')
        cv2.imwrite(path, frame)
        self.get_logger().info(f'Saved {path}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()