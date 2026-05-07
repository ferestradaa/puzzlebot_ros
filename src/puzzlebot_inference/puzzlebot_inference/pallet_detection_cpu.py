import os
import numpy as np
import cv2
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

PKG_SHARE = get_package_share_directory('puzzlebot_inference')
MODEL_PATH = os.path.join(PKG_SHARE, 'models', 'full_holes_raw.onnx')
ENGINE_CACHE_DIR = os.path.join(PKG_SHARE, 'models', 'trt_cache')

INPUT_H, INPUT_W = 320, 320
CONF_THRESH = 0.5
IOU_THRESH  = 0.1
NUM_CLASSES = 2  # solo pallet

CLASS_COLORS = {
    0: (0, 255, 0),
    1: (0, 0, 255),
}
CLASS_NAMES = {
    0: 'pallet',
    1: 'hole',
}


def build_session(model_path: str, cache_dir: str) -> ort.InferenceSession:
    os.makedirs(cache_dir, exist_ok=True)
    providers = ['CPUExecutionProvider']
    so = ort.SessionOptions()
    so.log_severity_level = 3
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    so.intra_op_num_threads = 4
    so.inter_op_num_threads = 4
    return ort.InferenceSession(model_path, sess_options=so, providers=providers)


class PalletNode(Node):
    def __init__(self):
        super().__init__('pallet_inference_cpu')
        self.bridge = CvBridge()
        self.get_logger().info(f'Modelo: {MODEL_PATH}')
        self.session = build_session(MODEL_PATH, ENGINE_CACHE_DIR)
        self.input_name = self.session.get_inputs()[0].name

        # Det tiene UNA sola salida: (1, 5, 8400) o (1, 4+nc, 8400)
        out = self.session.get_outputs()
        assert len(out) == 1, f'Esperaba 1 salida, got {len(out)}'
        self.output_name = out[0].name
        self.get_logger().info(f'Output: {out[0].name} {out[0].shape}')
        self.get_logger().info(f'Provider: {self.session.get_providers()[0]}')

        self._input_buf = np.zeros((1, 3, INPUT_H, INPUT_W), dtype=np.float32)

        # Publica BBoxes en vez de máscara mono8
        self.pub = self.create_publisher(Detection2DArray, '/pallet/detections', 10)
        # Opcional: imagen debug con boxes dibujados
        self.pub_dbg = self.create_publisher(Image, '/pallet/debug', 10)

        self.sub = self.create_subscription(
            Image, 'camera/image_raw', self.cb, 10)
        self.get_logger().info('Listo.')

    def preprocess(self, bgr):
        img = cv2.resize(bgr, (INPUT_W, INPUT_H))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self._input_buf[0] = img.transpose(2, 0, 1).astype(np.float32) / 255.0
        return self._input_buf

    def infer(self, bgr):
        inp = self.preprocess(bgr)
        # output shape: (1, 4+nc, num_anchors)  →  típicamente (1, 5, 8400)
        raw = self.session.run([self.output_name], {self.input_name: inp})[0]
        return raw  # (1, 5, 8400)

    def postprocess(self, raw, orig_w, orig_h):
        """
        raw: (1, 4+nc, 8400)
        Retorna lista de (x1,y1,x2,y2,conf,cls) en coords originales.
        """
        preds = raw[0].T  # (8400, 4+nc)

        # scores por clase
        boxes_xywh = preds[:, :4]
        class_scores = preds[:, 4:]  # (8400, nc)
        cls_ids = np.argmax(class_scores, axis=1)
        confs   = class_scores[np.arange(len(cls_ids)), cls_ids]

        keep = confs > CONF_THRESH
        
        if not keep.any():
            return []

        boxes_xywh = boxes_xywh[keep]
        confs       = confs[keep]
        cls_ids     = cls_ids[keep]

        sx, sy = orig_w / INPUT_W, orig_h / INPUT_H
        xc, yc, w, h = boxes_xywh.T
        x1 = ((xc - w / 2) * sx).clip(0, orig_w)
        y1 = ((yc - h / 2) * sy).clip(0, orig_h)
        x2 = ((xc + w / 2) * sx).clip(0, orig_w)
        y2 = ((yc + h / 2) * sy).clip(0, orig_h)
        boxes_for_nms = np.stack([x1, y1, x2 - x1, y2 - y1], axis=1)

        idx = cv2.dnn.NMSBoxes(
            boxes_for_nms.tolist(), confs.tolist(), CONF_THRESH, IOU_THRESH)
        if len(idx) == 0:
            return []

        idx = np.asarray(idx).flatten()
        results = []
        for i in idx:
            results.append((
                float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i]),
                float(confs[i]), int(cls_ids[i])
            ))
        return results

    def cb(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        orig_h, orig_w = bgr.shape[:2]

        try:
            raw = self.infer(bgr)
        except Exception as e:
            self.get_logger().error(f'Inferencia fallo: {e}')
            return

        detections = self.postprocess(raw, orig_w, orig_h)

        # --- Publicar Detection2DArray ---
        det_arr = Detection2DArray()
        det_arr.header = msg.header
        for (x1, y1, x2, y2, conf, cls) in detections:
            d = Detection2D()
            d.bbox.center.position.x = (x1 + x2) / 2.0
            d.bbox.center.position.y = (y1 + y2) / 2.0
            d.bbox.size_x = x2 - x1
            d.bbox.size_y = y2 - y1
            det_arr.detections.append(d)
        self.pub.publish(det_arr)

        # --- Debug visual ---
        if self.pub_dbg.get_subscription_count() > 0:
            dbg = bgr.copy()
            for (x1, y1, x2, y2, conf, cls) in detections:
                color = CLASS_COLORS.get(cls, (255, 255, 255))
                cv2.rectangle(dbg, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(dbg, f'{conf:.2f}', (int(x1), int(y1) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8'))


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






































































































































































