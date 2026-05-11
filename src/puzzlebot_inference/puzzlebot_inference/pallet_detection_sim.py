import os
import numpy as np
import cv2
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
from ament_index_python.packages import get_package_share_directory

PKG_SHARE = get_package_share_directory('puzzlebot_inference')
MODEL_PATH = os.path.join(PKG_SHARE, 'models/onnx', 'pallet_side.onnx')

INPUT_H, INPUT_W = 384, 384
CONF_THRESH = 0.5
IOU_THRESH  = 0.45
MASK_THRESH = 0.5
NUM_MASK_COEFFS = 32


def build_session(model_path: str) -> ort.InferenceSession:
    providers = [
        ('CUDAExecutionProvider', {'cudnn_conv_algo_search': 'HEURISTIC'}),
        'CPUExecutionProvider',
    ]
    so = ort.SessionOptions()
    so.log_severity_level = 3
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    so.intra_op_num_threads = 1
    so.inter_op_num_threads = 1
    return ort.InferenceSession(model_path, sess_options=so, providers=providers)


class PalletNode(Node):
    def __init__(self):
        super().__init__('pallet_detection_sim')
        self.bridge = CvBridge()
        self.enabled = False

        self.get_logger().info(f'Modelo: {MODEL_PATH}')
        self.session = build_session(MODEL_PATH)
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [o.name for o in self.session.get_outputs()]
        self.get_logger().info(f'Provider: {self.session.get_providers()[0]}')

        self._input_buf = np.zeros((1, 3, INPUT_H, INPUT_W), dtype=np.float32)

        self.srv = self.create_service(
            SetBool,
            'enable_detection',
            self.enable_callback
        )

        self.pub = self.create_publisher(Image, '/pallet/mask', 10)
        self.sub = self.create_subscription(
            Image, 'camera/image_raw', self.cb, 10)

        self.get_logger().info('Listo.')

    def enable_callback(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = f'Detection {"enabled" if self.enabled else "disabled"}'
        return response

    def preprocess(self, bgr):
        img = cv2.resize(bgr, (INPUT_W, INPUT_H))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self._input_buf[0] = img.transpose(2, 0, 1).astype(np.float32) / 255.0
        return self._input_buf

    def infer(self, bgr):
        inp = self.preprocess(bgr)
        outputs = self.session.run(self.output_names, {self.input_name: inp})
        out0 = out1 = None
        for o in outputs:
            if o.ndim == 3:
                out0 = o
            elif o.ndim == 4:
                out1 = o
        if out0 is None or out1 is None:
            raise RuntimeError(f'Salidas inesperadas: {[o.shape for o in outputs]}')
        return out0, out1

    def cb(self, msg: Image):
        if not self.enabled:
            return

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        orig_h, orig_w = bgr.shape[:2]
        empty = np.zeros((orig_h, orig_w), dtype=np.uint8)

        try:
            out0, out1 = self.infer(bgr)
        except Exception as e:
            self.get_logger().error(f'Inferencia fallo: {e}')
            self.pub.publish(self.bridge.cv2_to_imgmsg(empty, encoding='mono8'))
            return

        preds = out0[0].T
        confs = preds[:, 4]
        keep = confs > CONF_THRESH
        if not keep.any():
            self.pub.publish(self.bridge.cv2_to_imgmsg(empty, encoding='mono8'))
            return

        preds = preds[keep]
        confs = preds[:, 4]
        mask_coeffs = preds[:, 5:]

        sx, sy = orig_w / INPUT_W, orig_h / INPUT_H
        xc, yc, w, h = preds[:, 0], preds[:, 1], preds[:, 2], preds[:, 3]
        boxes = np.stack([
            (xc - w / 2) * sx, (yc - h / 2) * sy,
            (xc + w / 2) * sx, (yc + h / 2) * sy
        ], axis=1).clip([0, 0, 0, 0], [orig_w, orig_h, orig_w, orig_h])

        idx = cv2.dnn.NMSBoxes(
            boxes.tolist(), confs.tolist(), CONF_THRESH, IOU_THRESH)
        if len(idx) == 0:
            self.pub.publish(self.bridge.cv2_to_imgmsg(empty, encoding='mono8'))
            return

        idx = np.asarray(idx).flatten()[:3]
        boxes = boxes[idx]
        mask_coeffs = mask_coeffs[idx]

        protos = out1[0].reshape(NUM_MASK_COEFFS, -1)
        raw = mask_coeffs @ protos
        masks_proto = 1.0 / (1.0 + np.exp(-raw))

        _, _, proto_h, proto_w = out1.shape
        masks_proto = masks_proto.reshape(-1, proto_h, proto_w)
        combined_proto = np.max(masks_proto, axis=0)
        combined_proto = (combined_proto > MASK_THRESH).astype(np.uint8) * 255

        combined = cv2.resize(combined_proto, (orig_w, orig_h),
                              interpolation=cv2.INTER_NEAREST)
        self.pub.publish(self.bridge.cv2_to_imgmsg(combined, encoding='mono8'))


def main(args=None):
    rclpy.init(args=args)
    node = PalletNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()