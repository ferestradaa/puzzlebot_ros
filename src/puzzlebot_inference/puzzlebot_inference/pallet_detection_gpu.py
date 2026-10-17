import os
import platform
import time
import numpy as np
import cv2
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

PKG_SHARE = get_package_share_directory('puzzlebot_inference')
MODEL_PATH = os.path.join(PKG_SHARE, 'models', 'pallet_side.onnx')
ENGINE_CACHE_DIR = os.path.join(PKG_SHARE, 'models', 'trt_cache')

# Network input size (must match the ONNX export)
INPUT_H, INPUT_W = 384, 384
DEFAULT_CONF_THRESH = 0.25  # same default as the C++ detector this node replaces
IOU_THRESH  = 0.45



def use_tensorrt() -> bool:
    """TensorRT EP on the Jetson (aarch64); plain CUDA on desktop/sim, where the TRT attempt
    can fail on new GPUs with an old TensorRT and waste ~15 s. PALLET_USE_TRT=0/1 overrides."""
    env = os.environ.get('PALLET_USE_TRT')
    if env is not None:
        return env == '1'
    return platform.machine() == 'aarch64'


def build_session(model_path: str, cache_dir: str, logger) -> ort.InferenceSession:
    trt = use_tensorrt()
    providers = []
    if trt:
        os.makedirs(cache_dir, exist_ok=True)
        providers.append(('TensorrtExecutionProvider', {
            'trt_fp16_enable': True,
            'trt_engine_cache_enable': True,
            'trt_engine_cache_path': cache_dir,
            'trt_max_workspace_size': 128 * 1024 * 1024,
        }))
    providers += [
        ('CUDAExecutionProvider', {
            'cudnn_conv_algo_search': 'HEURISTIC',
        }),
        'CPUExecutionProvider',
    ]
    so = ort.SessionOptions()
    so.log_severity_level = 3  # errors only
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    # Limit CPU threads to leave cores for ROS 2 and navigation
    so.intra_op_num_threads = 1
    so.inter_op_num_threads = 1

    try:
        return ort.InferenceSession(model_path, sess_options=so, providers=providers)
    except Exception as e:
        if not trt:
            raise
        # TRT EP can fail while building the engine; retry without it
        logger.warn(f'TensorRT EP failed, falling back to CUDA/CPU: {e}')
        return ort.InferenceSession(
            model_path, sess_options=so, providers=providers[1:])


class PalletNode(Node):
    def __init__(self):
        super().__init__('pallet_inference_gpu')
        self.bridge = CvBridge()
        self.conf_thresh = self.declare_parameter(
            'conf_threshold', DEFAULT_CONF_THRESH).value

        t_load = time.monotonic()
        self.get_logger().info(
            f'Loading {os.path.basename(MODEL_PATH)} (TensorRT requested: {use_tensorrt()}, conf>{self.conf_thresh})')
        self.session = build_session(MODEL_PATH, ENGINE_CACHE_DIR, self.get_logger())

        self.input_name = self.session.get_inputs()[0].name
        # Detections are the 3-D output; the 4-D mask prototypes are not needed, so skip them
        self.det_output = next(o.name for o in self.session.get_outputs() if len(o.shape) == 3)

        outputs_desc = ', '.join(f'{o.name}{o.shape}' for o in self.session.get_outputs())
        self.get_logger().info(
            f'Model loaded in {time.monotonic() - t_load:.2f} s | '
            f'provider={self.session.get_providers()[0]} | outputs: {outputs_desc}')

        # Reusable input buffer, avoids an allocation per frame
        self._input_buf = np.zeros((1, 3, INPUT_H, INPUT_W), dtype=np.float32)

        # Camera frame with the detections drawn on it
        self.debug_pub = self.create_publisher(Image, '/pallet_detections/image', 10)
        # Best pallet box (image pixels), consumed by visual_servoing
        self.det_pub = self.create_publisher(
            Detection2DArray, '/pallet_inference_centroid', 1)
        self.sub = self.create_subscription(
            Image, 'camera/image_raw', self.cb, 10)

        # Warmup: the first run initializes the CUDA/TRT context, do it before reporting ready
        t_warm = time.monotonic()
        self.infer(np.zeros((INPUT_H, INPUT_W, 3), dtype=np.uint8))
        warmup_s = time.monotonic() - t_warm

        # transient_local so the process_manager gets it even if it subscribes late
        ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.ready_pub = self.create_publisher(
            Bool, '/pallet_detector/ready', ready_qos)
        self.ready_pub.publish(Bool(data=True))
        self.get_logger().info(
            f'Pallet detector ready (warmup {warmup_s:.2f} s): '
            f'camera/image_raw -> /pallet_detections/image + /pallet_inference_centroid, '
            f'ready flag on /pallet_detector/ready')

    def preprocess(self, bgr):
        """Preprocess in place into the reusable input buffer."""
        img = cv2.resize(bgr, (INPUT_W, INPUT_H))
        # BGR -> RGB, then normalize with a vectorized division
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        np.divide(img, 255.0, out=self._input_buf[0].transpose(1, 2, 0),
                  casting='unsafe')
        # Make sure the buffer is contiguous CHW
        self._input_buf[0] = img.transpose(2, 0, 1).astype(np.float32) / 255.0
        return self._input_buf

    def infer(self, bgr):
        inp = self.preprocess(bgr)
        return self.session.run([self.det_output], {self.input_name: inp})[0]

    def cb(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        orig_h, orig_w = bgr.shape[:2]

        def publish_debug(frame):
            out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out.header = msg.header
            self.debug_pub.publish(out)

        def publish_none():
            publish_debug(bgr)
            self.det_pub.publish(Detection2DArray(header=msg.header))

        try:
            out0 = self.infer(bgr)
        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}', throttle_duration_sec=5.0)
            publish_none()
            return

        preds = out0[0].T
        confs = preds[:, 4]
        keep = confs > self.conf_thresh
        if not keep.any():
            self.get_logger().info(
                f'No detection above {self.conf_thresh} (best raw conf {confs.max():.2f})',
                throttle_duration_sec=2.0)
            publish_none()
            return

        preds = preds[keep]
        confs = preds[:, 4]

        sx, sy = orig_w / INPUT_W, orig_h / INPUT_H
        xc, yc, w, h = preds[:, 0], preds[:, 1], preds[:, 2], preds[:, 3]
        boxes = np.stack([
            (xc - w / 2) * sx, (yc - h / 2) * sy,
            (xc + w / 2) * sx, (yc + h / 2) * sy
        ], axis=1).clip([0, 0, 0, 0], [orig_w, orig_h, orig_w, orig_h])

        idx = cv2.dnn.NMSBoxes(
            boxes.tolist(), confs.tolist(), self.conf_thresh, IOU_THRESH)
        if len(idx) == 0:
            publish_none()
            return

        # NMSBoxes returns indices sorted by score; keep the best few
        idx = np.asarray(idx).flatten()[:3]
        boxes, confs = boxes[idx], confs[idx]

        bx1, by1, bx2, by2 = boxes[0]
        self.get_logger().info(
            f'{len(idx)} pallet(s), best conf {confs[0]:.2f}, '
            f'box {bx2 - bx1:.0f}x{by2 - by1:.0f} px (area {(bx2 - bx1) * (by2 - by1):.0f})',
            throttle_duration_sec=2.0)

        annotated = bgr.copy()
        for i, (box, conf) in enumerate(zip(boxes, confs)):
            x1, y1, x2, y2 = (int(round(v)) for v in box)
            # Best detection (the one visual_servoing follows) in green, others in yellow
            color = (0, 255, 0) if i == 0 else (0, 255, 255)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated, f'pallet {conf:.2f}', (x1, max(y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        publish_debug(annotated)
        self.publish_best_box(msg.header, boxes[0], float(confs[0]))

    def publish_best_box(self, header, box, conf):
        x1, y1, x2, y2 = (float(v) for v in box)
        det = Detection2D(header=header)
        det.bbox.center.position.x = (x1 + x2) / 2.0
        det.bbox.center.position.y = (y1 + y2) / 2.0
        det.bbox.size_x = x2 - x1
        det.bbox.size_y = y2 - y1
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = 'pallet'
        hyp.hypothesis.score = conf
        det.results.append(hyp)
        self.det_pub.publish(Detection2DArray(header=header, detections=[det]))


def main(args=None):
    rclpy.init(args=args)
    node = PalletNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # SIGINT/SIGTERM (e.g. KillNode) shuts the context down while a callback may still be
        # publishing; that race is expected on shutdown, anything else is a real error
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

