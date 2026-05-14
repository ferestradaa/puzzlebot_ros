import os
import numpy as np
import cv2
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2DArray, Detection2D 
from ament_index_python.packages import get_package_share_directory

PKG_SHARE = get_package_share_directory('puzzlebot_inference')
MODEL_PATH = os.path.join(PKG_SHARE, 'models', 'ft2_full_holes.onnx')

INPUT_H, INPUT_W = 320, 320
CONF_THRESH = 0.5
IOU_THRESH  = 0.45
MAX_DET     = 3


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

        self.srv = self.create_service(SetBool, 'enable_detection', self.enable_callback)
        self.pub = self.create_publisher(Detection2DArray, '/pallet/detections', 10)
        self.pub_viz = self.create_publisher(Image, '/pallet/image_detections', 10)
        
        self.sub = self.create_subscription(Image, 'camera/image_raw', self.cb, 10)
        self.get_logger().info('Ready.')

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
        out = outputs[0]
        if out.ndim != 3:
            raise RuntimeError(f'out: {out.shape}')
        return out

    def publish_empty(self, header):
        msg = Detection2DArray()
        msg.header = header
        self.pub.publish(msg)

    def cb(self, msg: Image):
        if not self.enabled:
            pass
            # return

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        orig_h, orig_w = bgr.shape[:2]

        try:
            out = self.infer(bgr)
        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')
            self.publish_empty(msg.header)
            return

        preds = out[0].T
        num_classes = preds.shape[1] - 4
        if num_classes < 1:
            self.get_logger().error(f'insufiicnet channels: {preds.shape}')
            self.publish_empty(msg.header)
            return

        boxes_xywh = preds[:, :4]
        class_scores = preds[:, 4:4 + num_classes]
        confs = class_scores.max(axis=1)
        class_ids = class_scores.argmax(axis=1)

        keep = confs > CONF_THRESH
        if not keep.any():
            self.publish_empty(msg.header)
            return

        boxes_xywh = boxes_xywh[keep]
        confs = confs[keep]
        class_ids = class_ids[keep]

        sx, sy = orig_w / INPUT_W, orig_h / INPUT_H
        xc, yc, w, h = boxes_xywh[:, 0], boxes_xywh[:, 1], boxes_xywh[:, 2], boxes_xywh[:, 3]
        boxes = np.stack([
            (xc - w / 2) * sx, (yc - h / 2) * sy,
            (xc + w / 2) * sx, (yc + h / 2) * sy
        ], axis=1).clip([0, 0, 0, 0], [orig_w, orig_h, orig_w, orig_h])

        idx = cv2.dnn.NMSBoxes(boxes.tolist(), confs.tolist(), CONF_THRESH, IOU_THRESH)
        if len(idx) == 0:
            self.publish_empty(msg.header)
            return

        idx = np.asarray(idx).flatten()[:MAX_DET]

        det_array = Detection2DArray()
        det_array.header = msg.header

        for i in idx:
            x1, y1, x2, y2 = boxes[i]
            det = Detection2D()
            det.bbox.center.position.x = float((x1 + x2) * 0.5)
            det.bbox.center.position.y = float((y1 + y2) * 0.5)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            #hyp = ObjectHypothesisWithPose()
            #hyp.hypothesis.class_id = str(int(class_ids[i]))
            #hyp.hypothesis.score = float(confs[i])
            #det.results.append(hyp)

            det_array.detections.append(det)

        self.pub.publish(det_array)

        for i in idx:
            x1, y1, x2, y2 = boxes[i].astype(int)
            cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)

        viz_msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        viz_msg.header = msg.header
        self.pub_viz.publish(viz_msg)

        


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