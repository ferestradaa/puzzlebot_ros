import rclpy
import rclpy.time
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped
from pupil_apriltags import Detector
from puzzlebot_interfaces.msg import AprilTagDetection, AprilTagDetectionArray
import cv2


import os
import sys


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        self.bridge = CvBridge()
        self.camera_matrix = None

        self.declare_parameter('tag_size', 0.125)
        self.declare_parameter('base_frame', 'base_link')
        self.tag_size = self.get_parameter('tag_size').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.detector = Detector(
            families='tag36h11',
            nthreads=2,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
        )

        self.sub_image = self.create_subscription(Image, 'camera/image_raw', self.image_cb, qos)
        self.sub_info = self.create_subscription(CameraInfo, 'camera/camera_info', self.info_cb, qos)
        self.pub_camera = self.create_publisher(AprilTagDetectionArray, 'apriltag/camera_pose', 10)
        self.pub_base = self.create_publisher(AprilTagDetectionArray, 'apriltag/base_pose', 10)

        self.get_logger().info("Apriltag node initializad")

    def info_cb(self, msg: CameraInfo):
        if self.camera_matrix is not None:
            return
        k = msg.k
        self.camera_matrix = np.array([
            [k[0], k[1], k[2]],
            [k[3], k[4], k[5]],
            [k[6], k[7], k[8]],
        ], dtype=np.float64)
        self.get_logger().info('Camera info recieved')

    def _get_extrinsic(self, camera_frame):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame=self.base_frame,
                source_frame=camera_frame,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'No TF {self.base_frame} to {camera_frame}: {e}')
            return None

    def image_cb(self, msg: Image):
        if self.camera_matrix is None:
            return

        extrinsic = self._get_extrinsic(msg.header.frame_id)

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        #frame = self._preprocess(frame)  

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        devnull = os.open(os.devnull, os.O_WRONLY)
        old_stderr = os.dup(2)

        try:
            os.dup2(devnull, 2)

            detections = self.detector.detect(
                frame,
                estimate_tag_pose=True,
                camera_params=(fx, fy, cx, cy),
                tag_size=self.tag_size,
            )

        finally:
            os.dup2(old_stderr, 2)
            os.close(old_stderr)
            os.close(devnull)

        cam_array = AprilTagDetectionArray()
        cam_array.header = msg.header

        base_array = AprilTagDetectionArray()
        base_array.header.stamp = msg.header.stamp
        base_array.header.frame_id = self.base_frame

        for det in detections:
            pose_cam = self._build_pose(det, msg.header)

            cam_det = AprilTagDetection()
            cam_det.tag_id = int(det.tag_id)
            cam_det.pose = pose_cam
            cam_array.detections.append(cam_det)

            self._broadcast_tf(pose_cam.pose, msg.header.stamp,
                               msg.header.frame_id, f'tag_{det.tag_id}')

            if extrinsic is not None:
                pose_base_raw = tf2_geometry_msgs.do_transform_pose(pose_cam.pose, extrinsic)
                pose_base = PoseStamped()
                pose_base.header.stamp = msg.header.stamp
                pose_base.header.frame_id = self.base_frame
                pose_base.pose = pose_base_raw

                base_det = AprilTagDetection()
                base_det.tag_id = int(det.tag_id)
                base_det.pose = pose_base
                base_array.detections.append(base_det)

        self.pub_camera.publish(cam_array)
        if base_array.detections:
            self.pub_base.publish(base_array)

    def _broadcast_tf(self, pose, stamp, parent_frame, child_frame):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = pose.position.x
        tf_msg.transform.translation.y = pose.position.y
        tf_msg.transform.translation.z = pose.position.z
        tf_msg.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def _build_pose(self, det, header) -> PoseStamped:
        pose = PoseStamped()
        pose.header = header

        t = det.pose_t.flatten()


        pose.pose.position.x = float(t[0])
        pose.pose.position.y = float(t[1])
        pose.pose.position.z = float(t[2])

        q = self.mat_to_quat(det.pose_R)
        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])

        return pose

    def mat_to_quat(self, R):
        t = np.trace(R)
        if t > 0:
            s = np.sqrt(t + 1.0) * 2
            w = 0.25 * s
            x = (R[2,1] - R[1,2]) / s
            y = (R[0,2] - R[2,0]) / s
            z = (R[1,0] - R[0,1]) / s
        elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / s
            x = 0.25 * s
            y = (R[0,1] + R[1,0]) / s
            z = (R[0,2] + R[2,0]) / s
        elif R[1,1] > R[2,2]:
            s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / s
            x = (R[0,1] + R[1,0]) / s
            y = 0.25 * s
            z = (R[1,2] + R[2,1]) / s
        else:
            s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / s
            x = (R[0,2] + R[2,0]) / s
            y = (R[1,2] + R[2,1]) / s
            z = 0.25 * s
        return [x, y, z, w]
    

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        min_val = np.percentile(frame, 2)
        max_val = np.percentile(frame, 98)
        stretched = np.clip((frame - min_val) / (max_val - min_val + 1e-6) * 255, 0, 255).astype(np.uint8)

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return clahe.apply(stretched)


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