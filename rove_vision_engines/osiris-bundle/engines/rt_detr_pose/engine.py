"""
RT-DETR + ViTPose Engine

Two-stage pose estimation:
1. RT-DETR detects person bounding boxes
2. ViTPose+ predicts 17 COCO keypoints for each detected person

Both models are Apache 2.0 licensed (PekingU RT-DETR, USyd ViTPose+).
Uses HuggingFace Transformers for inference.

Outputs detections with:
- bbox: [x, y, w, h] person bounding box
- class: "person"
- confidence: detection score
- keypoints: list of 17 [x, y, visibility] (COCO format)
"""

import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "templates", "engine_template"))
from engine import OsirisEngine, main

# COCO 17 keypoints
KEYPOINT_NAMES = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
]


class RTDETRPoseEngine(OsirisEngine):
    """RT-DETR detection + ViTPose+ keypoint estimation."""

    def __init__(self):
        super().__init__()
        self.detector = None
        self.detector_processor = None
        self.pose_model = None
        self.pose_processor = None
        self.device = "cpu"
        self.confidence_threshold = 0.5
        self.max_detections = 100

    def initialize(self, config: dict) -> dict:
        try:
            import torch
            from transformers import (
                AutoProcessor,
                RTDetrForObjectDetection,
                VitPoseForPoseEstimation,
            )
        except ImportError as e:
            print(f"WARNING: Missing dependencies: {e}", file=sys.stderr)
            print("Install: pip install torch transformers", file=sys.stderr)
            self.classes = ["person"]
            return {
                "classes": self.classes,
                "input_size": [640, 640],
                "provider": "none (missing deps)",
                "keypoints": KEYPOINT_NAMES,
            }

        # Device
        device_config = config.get("device", "auto")
        if device_config == "cpu":
            self.device = "cpu"
        elif device_config.startswith("cuda") and torch.cuda.is_available():
            self.device = device_config
        elif torch.cuda.is_available():
            self.device = "cuda:0"
        else:
            self.device = "cpu"

        self.confidence_threshold = config.get("confidence_threshold", 0.5)
        self.max_detections = config.get("max_detections", 100)
        self.classes = ["person"]

        # Load detector (RT-DETR pretrained on COCO + Objects365)
        detector_id = config.get("detector_model", "PekingU/rtdetr_r50vd_coco_o365")
        print(f"Loading detector: {detector_id}", file=sys.stderr)
        self.detector_processor = AutoProcessor.from_pretrained(detector_id)
        self.detector = RTDetrForObjectDetection.from_pretrained(detector_id).to(self.device).eval()

        # Load pose model (ViTPose+)
        pose_id = config.get("pose_model", "usyd-community/vitpose-plus-small")
        print(f"Loading pose model: {pose_id}", file=sys.stderr)
        self.pose_processor = AutoProcessor.from_pretrained(pose_id)
        self.pose_model = VitPoseForPoseEstimation.from_pretrained(pose_id).to(self.device).eval()

        return {
            "classes": self.classes,
            "input_size": [640, 640],
            "provider": self.device,
            "keypoints": KEYPOINT_NAMES,
            "detector": detector_id,
            "pose_model": pose_id,
        }

    def process_frame(self, feed_id, frame, width, height, channels):
        if self.detector is None or self.pose_model is None:
            return []

        import torch

        # BGR -> RGB
        img_bgr = np.frombuffer(frame, dtype=np.uint8).reshape((height, width, channels))
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        # Stage 1: detect persons with RT-DETR
        with torch.no_grad():
            inputs = self.detector_processor(images=img_rgb, return_tensors="pt").to(self.device)
            outputs = self.detector(**inputs)

            target_sizes = torch.tensor([(height, width)], device=self.device)
            results = self.detector_processor.post_process_object_detection(
                outputs,
                target_sizes=target_sizes,
                threshold=self.confidence_threshold,
            )[0]

        # Filter to "person" class only (COCO label 0)
        person_boxes = []
        person_scores = []
        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            if int(label) == 0:  # person
                x1, y1, x2, y2 = [float(c) for c in box.cpu().tolist()]
                # Convert to xywh format expected by ViTPose
                person_boxes.append([x1, y1, x2 - x1, y2 - y1])
                person_scores.append(float(score))

        if not person_boxes:
            return []

        # Stage 2: estimate pose for each person
        with torch.no_grad():
            pose_inputs = self.pose_processor(
                img_rgb,
                boxes=[person_boxes],
                return_tensors="pt",
            ).to(self.device)

            # ViTPose+ uses MoE — pass dataset_index=0 for COCO expert
            # (one index per person box)
            num_persons = len(person_boxes)
            dataset_index = torch.zeros(num_persons, dtype=torch.long, device=self.device)

            pose_outputs = self.pose_model(**pose_inputs, dataset_index=dataset_index)

            pose_results = self.pose_processor.post_process_pose_estimation(
                pose_outputs,
                boxes=[person_boxes],
            )[0]

        detections = []
        for i, (box, score, kp_data) in enumerate(zip(person_boxes, person_scores, pose_results)):
            keypoints = []
            kp_xy = kp_data.get("keypoints", [])
            kp_scores = kp_data.get("scores", [])

            # Convert tensors to lists if needed
            if hasattr(kp_xy, "cpu"):
                kp_xy = kp_xy.cpu().tolist()
            if hasattr(kp_scores, "cpu"):
                kp_scores = kp_scores.cpu().tolist()

            for j in range(min(17, len(kp_xy))):
                kx, ky = float(kp_xy[j][0]), float(kp_xy[j][1])
                kv = float(kp_scores[j]) if j < len(kp_scores) else 1.0
                keypoints.append([round(kx, 1), round(ky, 1), round(kv, 3)])

            detections.append({
                "class": "person",
                "confidence": round(score, 4),
                "bbox": [round(box[0], 1), round(box[1], 1), round(box[2], 1), round(box[3], 1)],
                "keypoints": keypoints,
                "track_id": None,
            })

        detections.sort(key=lambda d: d["confidence"], reverse=True)
        return detections[: self.max_detections]


if __name__ == "__main__":
    main(RTDETRPoseEngine)
