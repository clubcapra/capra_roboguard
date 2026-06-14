"""Auto-generated trained RT-DETR engine."""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "templates", "engine_template"))
from engine import OsirisEngine, main


class TrainedRTDETREngine(OsirisEngine):
    def __init__(self):
        super().__init__()
        self.model = None
        self.processor = None
        self.device = "cpu"
        self.confidence_threshold = 0.25
        self.max_detections = 100
        self.temperature = 1.0

    def initialize(self, config: dict) -> dict:
        try:
            import torch
            from transformers import AutoImageProcessor, AutoModelForObjectDetection
        except ImportError as e:
            print(f"WARNING: Missing deps: {e}", file=sys.stderr)
            self.classes = config.get("classes", [])
            return {"classes": self.classes, "input_size": [640, 640], "provider": "none"}

        device = config.get("device", "auto")
        if device == "auto":
            self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device

        self.confidence_threshold = config.get("confidence_threshold", self.confidence_threshold)
        self.max_detections = config.get("max_detections", 100)
        self.temperature = float(config.get("temperature", 1.0)) or 1.0
        self.classes = config.get("classes", [])

        model_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), config.get("weights", "model"))
        self.processor = AutoImageProcessor.from_pretrained(model_dir)
        self.model = AutoModelForObjectDetection.from_pretrained(model_dir).to(self.device).eval()

        return {
            "classes": self.classes,
            "input_size": list(config.get("input_size", [640, 640])),
            "provider": self.device,
        }

    def process_frame(self, feed_id, frame, width, height, channels):
        if self.model is None:
            return []

        import torch
        img_bgr = np.frombuffer(frame, dtype=np.uint8).reshape((height, width, channels))
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        with torch.no_grad():
            inputs = self.processor(images=img_rgb, return_tensors="pt").to(self.device)
            outputs = self.model(**inputs)
            if self.temperature != 1.0 and getattr(outputs, "logits", None) is not None:
                outputs.logits = outputs.logits / self.temperature
            target_sizes = torch.tensor([(height, width)], device=self.device)
            results = self.processor.post_process_object_detection(
                outputs, target_sizes=target_sizes, threshold=self.confidence_threshold,
            )[0]

        detections = []
        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            cls_id = int(label)
            if cls_id < 0 or cls_id >= len(self.classes):
                continue
            x1, y1, x2, y2 = [float(c) for c in box.cpu().tolist()]
            detections.append({
                "class": self.classes[cls_id],
                "confidence": round(float(score), 4),
                "bbox": [round(x1, 1), round(y1, 1), round(x2 - x1, 1), round(y2 - y1, 1)],
                "track_id": None,
            })

        detections.sort(key=lambda d: d["confidence"], reverse=True)
        return detections[: self.max_detections]


if __name__ == "__main__":
    main(TrainedRTDETREngine)
