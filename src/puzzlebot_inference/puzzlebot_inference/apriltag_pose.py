#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
import tf2_geometry_msgs

from pupil_apriltags import Detector


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        self.declare_parameter('tag_size', 0.1)
        self.tag_size = self.get_parameter('tag_size').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.detector = Detector(
            families='tag36h11',
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
        )

        self.sub_image = self.create_subscription(
            Image, 'camera/image_raw', self.image_cb, qos)
        self.sub_info = self.create_subscription(
            CameraInfo, 'camera/camera_info', self.info_cb, qos)

        self.pub_pose_cam = self.create_publisher(PoseStamped, 'apriltag/pose', 10)
        self.pub_pose_base = self.create_publisher(PoseStamped, 'apriltag/pose_base_', 10)

    def info_cb(self, msg: CameraInfo):
        if self.camera_matrix is not None:
            return
        k = msg.k
        self.camera_matrix = np.array([
            [k[0], k[1], k[2]],
            [k[3], k[4], k[5]],
            [k[6], k[7], k[8]],
        ], dtype=np.float64)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.get_logger().info('Camera info recibida')

    def image_cb(self, msg: Image):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        detections = self.detector.detect(
            frame,
            estimate_tag_pose=True,
            camera_params=(fx, fy, cx, cy),
            tag_size=self.tag_size,
        )

        for det in detections:
            pose_cam = self._build_pose(det, msg.header)
            self.pub_pose_cam.publish(pose_cam)

            self._publish_in_base_footprint(pose_cam)

    def _publish_in_base_footprint(self, pose_cam: PoseStamped):
        try:
            pose_cam.header.stamp = rclpy.time.Time().to_msg()
            pose_base = self.tf_buffer.transform(
                pose_cam,
                'base_link',
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            self.pub_pose_base.publish(pose_base)
            self.get_logger().info(
                f'base_link | '
                f'x={pose_base.pose.position.x:.3f} '
                f'y={pose_base.pose.position.y:.3f} '
                f'z={pose_base.pose.position.z:.3f}'
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF error: {e}')

    def _build_pose(self, det, header) -> PoseStamped:
        R = det.pose_R
        t = det.pose_t.flatten()

        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = float(t[0])
        pose.pose.position.y = float(t[1])
        pose.pose.position.z = float(t[2])
        pose.pose.orientation.x = float(x)
        pose.pose.orientation.y = float(y)
        pose.pose.orientation.z = float(z)
        pose.pose.orientation.w = float(w)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()