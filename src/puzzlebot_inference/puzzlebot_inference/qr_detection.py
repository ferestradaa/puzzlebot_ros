import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from pyzbar.pyzbar import decode

class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector')
        
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            qos)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            qos)
        
        self.publisher = self.create_publisher(Image, 'qr/detection', 10)
        self.bridge = CvBridge()
        
        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.last_detection = None
        self.detection_count = 0
        self.confirmation_threshold = 3
        
        self.qr_size = 0.10  # Tamaño real del QR en metros (ajusta según tu QR)
    
    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera info recibida')
    
    def estimate_pose(self, barcode):
        if self.camera_matrix is None:
            return None, None
        
        points = barcode.polygon
        if len(points) != 4:
            return None, None
        
        image_points = np.array([
            [points[0].x, points[0].y],
            [points[1].x, points[1].y],
            [points[2].x, points[2].y],
            [points[3].x, points[3].y]
        ], dtype=np.float32)
        
        half_size = self.qr_size / 2.0
        object_points = np.array([
            [-half_size, -half_size, 0],
            [half_size, -half_size, 0],
            [half_size, half_size, 0],
            [-half_size, half_size, 0]
        ], dtype=np.float32)
        
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs
        )
        
        if success:
            return rvec, tvec
        return None, None
    
    def preprocess_variants(self, gray):
        variants = []
        
        variants.append(gray)
        
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        variants.append(clahe.apply(gray))
        
        kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        sharpened = cv2.filter2D(gray, -1, kernel)
        variants.append(sharpened)
        
        bilateral = cv2.bilateralFilter(gray, 9, 75, 75)
        variants.append(bilateral)
        
        _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        variants.append(otsu)
        
        adaptive1 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                          cv2.THRESH_BINARY, 11, 2)
        variants.append(adaptive1)
        
        adaptive2 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                          cv2.THRESH_BINARY, 15, 3)
        variants.append(adaptive2)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        closed = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)
        variants.append(closed)
        
        return variants
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        height, width = cv_image.shape[:2]
        if width > 1280:
            scale = 1280 / width
            cv_image = cv2.resize(cv_image, None, fx=scale, fy=scale)
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        
        results = []
        variants = self.preprocess_variants(gray)
        
        for variant in variants:
            detected = decode(variant)
            results.extend(detected)
        
        output = cv_image.copy()
        seen = {}
        
        for barcode in results:
            data = barcode.data.decode('utf-8')
            if data not in seen:
                seen[data] = barcode
        
        if seen:
            current_data = list(seen.keys())[0]
            
            if self.last_detection == current_data:
                self.detection_count += 1
            else:
                self.last_detection = current_data
                self.detection_count = 1
            
            if self.detection_count >= self.confirmation_threshold:
                for data, barcode in seen.items():
                    points = barcode.polygon
                    if len(points) == 4:
                        pts = [(point.x, point.y) for point in points]
                        pts = np.array(pts, dtype=np.int32)
                        cv2.polylines(output, [pts], True, (0, 255, 0), 3)
                        
                        rvec, tvec = self.estimate_pose(barcode)
                        
                        x, y = barcode.rect.left, barcode.rect.top
                        
                        if tvec is not None:
                            cv2.drawFrameAxes(output, self.camera_matrix, self.dist_coeffs, 
                                            rvec, tvec, self.qr_size * 0.5)
                            
                            pos_text = f'Pos: [{tvec[0][0]:.3f}, {tvec[1][0]:.3f}, {tvec[2][0]:.3f}]m'
                            cv2.putText(output, data, (x, y - 30), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            cv2.putText(output, pos_text, (x, y - 10), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            
                            self.get_logger().info(f'QR: {current_data} | Pose: x={tvec[0][0]:.3f}, y={tvec[1][0]:.3f}, z={tvec[2][0]:.3f}')
                        else:
                            cv2.putText(output, f'{data} (conf:{self.detection_count})', 
                                       (x, y - 10), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            self.detection_count = 0
            self.last_detection = None
        
        output_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
        output_msg.header = msg.header
        self.publisher.publish(output_msg)

def main():
    rclpy.init()
    node = QRDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()