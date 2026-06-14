"""
RT-DETR COCO Vision Engine

Reference implementation of an Osiris vision engine using RT-DETR
with ONNX Runtime for inference on COCO-class objects.
"""

import os
import sys

import cv2
import numpy as np

# Add the template to path so we can import the base class
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "templates", "engine_template"))
from engine import OsirisEngine, main


class RTDETREngine(OsirisEngine):
    """RT-DETR object detection engine using ONNX Runtime."""

    def __init__(self):
        super().__init__()
        self.session = None
        self.classes = []
        self.input_size = (640, 640)
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4
        self.max_detections = 100

    def initialize(self, config: dict) -> dict:
        import onnxruntime as ort

        # Load CUDA/cuDNN from the nvidia-*-cu12 pip packages installed by the
        # GPU-aware installer, so the CUDAExecutionProvider can actually load
        # (otherwise onnxruntime silently falls back to CPU). No-op on CPU boxes
        # and on onnxruntime versions without preload_dlls().
        if hasattr(ort, "preload_dlls"):
            try:
                ort.preload_dlls()
            except Exception as e:  # noqa: BLE001 — best effort; CPU still works
                print(f"onnxruntime preload_dlls failed: {e}", file=sys.stderr)

        model_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(model_dir, config.get("weights", "model.onnx"))

        if not os.path.exists(weights_path):
            print(f"WARNING: Model weights not found at {weights_path}", file=sys.stderr)
            print("Engine will report ready but return empty detections.", file=sys.stderr)
            print("To add weights: download an RT-DETR ONNX model and place it as model.onnx", file=sys.stderr)
            self.session = None
            self.classes = config.get("classes", COCO_CLASSES)
            self.input_size = tuple(config.get("input_size", [640, 640]))
            return {
                "classes": self.classes,
                "input_size": list(self.input_size),
                "provider": "none (no weights)",
            }

        # Select execution provider
        device = config.get("device", "auto")
        providers = self._select_providers(device)

        self.session = ort.InferenceSession(weights_path, providers=providers)

        # Read config
        input_size = config.get("input_size", [640, 640])
        self.input_size = (input_size[0], input_size[1])
        self.confidence_threshold = config.get("confidence_threshold", 0.5)
        self.nms_threshold = config.get("nms_threshold", 0.4)
        self.max_detections = config.get("max_detections", 100)

        # Load classes
        self.classes = config.get("classes", [])
        if not self.classes:
            classes_file = os.path.join(model_dir, "classes.txt")
            if os.path.exists(classes_file):
                with open(classes_file) as f:
                    self.classes = [line.strip() for line in f if line.strip()]

        if not self.classes:
            self.classes = COCO_CLASSES

        # Get input/output names
        self.input_names = [inp.name for inp in self.session.get_inputs()]
        self.output_names = [out.name for out in self.session.get_outputs()]

        return {
            "classes": self.classes,
            "input_size": list(self.input_size),
            "provider": self.session.get_providers()[0],
        }

    def process_frame(
        self,
        feed_id: str,
        frame: bytes,
        width: int,
        height: int,
        channels: int,
    ) -> list[dict]:
        if self.session is None:
            return []

        # Decode raw BGR bytes into numpy array
        img = np.frombuffer(frame, dtype=np.uint8).reshape((height, width, channels))

        # Preprocess: resize, normalize, transpose to NCHW
        input_tensor, scale_x, scale_y = self._preprocess(img)

        # Run inference
        # RT-DETR typically expects: images (NCHW float32), orig_target_sizes (Nx2 int64)
        feeds = {}
        if len(self.input_names) == 1:
            feeds[self.input_names[0]] = input_tensor
        else:
            feeds[self.input_names[0]] = input_tensor
            # orig_target_sizes for RT-DETR post-processing
            orig_sizes = np.array([[height, width]], dtype=np.int64)
            if len(self.input_names) > 1:
                feeds[self.input_names[1]] = orig_sizes

        outputs = self.session.run(self.output_names, feeds)

        # Parse detections
        detections = self._postprocess(outputs, width, height, scale_x, scale_y)
        return detections

    def _preprocess(self, img: np.ndarray) -> tuple[np.ndarray, float, float]:
        """Resize, normalize, and convert to NCHW float32 tensor."""
        h, w = img.shape[:2]
        target_w, target_h = self.input_size

        # Resize
        resized = cv2.resize(img, (target_w, target_h), interpolation=cv2.INTER_LINEAR)

        scale_x = w / target_w
        scale_y = h / target_h

        # BGR to RGB
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        # Normalize to [0, 1]
        normalized = rgb.astype(np.float32) / 255.0

        # HWC to NCHW
        tensor = np.transpose(normalized, (2, 0, 1))[np.newaxis, ...]

        return tensor, scale_x, scale_y

    def _postprocess(
        self,
        outputs: list[np.ndarray],
        orig_w: int,
        orig_h: int,
        scale_x: float,
        scale_y: float,
    ) -> list[dict]:
        """Parse RT-DETR outputs into detection dicts."""
        detections = []
        num_classes = len(self.classes)

        preds = outputs[0]  # (1, num_queries, 4+num_classes)
        if preds.ndim == 3:
            preds = preds[0]  # (num_queries, 4+num_classes)

        for pred in preds:
            # First 4 values: cx, cy, w, h (normalized 0-1)
            cx, cy, w, h = float(pred[0]), float(pred[1]), float(pred[2]), float(pred[3])

            # Remaining values: per-class scores
            class_scores = pred[4:]
            class_id = int(np.argmax(class_scores))
            score = float(class_scores[class_id])

            if score < self.confidence_threshold:
                continue
            if class_id < 0 or class_id >= num_classes:
                continue

            # Convert normalized center coords to absolute top-left pixel coords
            x1 = float((cx - w / 2) * orig_w)
            y1 = float((cy - h / 2) * orig_h)
            bw = float(w * orig_w)
            bh = float(h * orig_h)

            detections.append({
                "class": self.classes[class_id],
                "confidence": round(score, 4),
                "bbox": [round(x1, 1), round(y1, 1), round(bw, 1), round(bh, 1)],
                "track_id": None,
            })

        # Sort by confidence and limit
        detections.sort(key=lambda d: d["confidence"], reverse=True)
        return detections[: self.max_detections]

    @staticmethod
    def _select_providers(device: str) -> list[str]:
        """Select ONNX Runtime execution providers based on device config."""
        import onnxruntime as ort

        available = ort.get_available_providers()

        if device == "cpu":
            return ["CPUExecutionProvider"]
        elif device.startswith("cuda"):
            if "CUDAExecutionProvider" in available:
                return ["CUDAExecutionProvider", "CPUExecutionProvider"]
            return ["CPUExecutionProvider"]
        else:  # auto
            if "CUDAExecutionProvider" in available:
                return ["CUDAExecutionProvider", "CPUExecutionProvider"]
            if "TensorrtExecutionProvider" in available:
                return ["TensorrtExecutionProvider", "CPUExecutionProvider"]
            return ["CPUExecutionProvider"]


# COCO 80 classes
COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]


if __name__ == "__main__":
    main(RTDETREngine)
