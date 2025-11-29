"""
YOLOv11 Object Detector Module

Provides real-time object detection using Ultralytics YOLOv11.
Integrates with DroneBrain pipeline for drone-based vision applications.
"""

import logging
from typing import List, Dict, Any, Optional, Tuple
import numpy as np
import cv2
from ultralytics import YOLO
import torch

from src.core.state import SystemState, Detection

logger = logging.getLogger(__name__)


class YOLODetector:
    """
    YOLOv11-based object detector for drone vision applications.

    Features:
    - GPU-accelerated inference
    - Configurable confidence threshold
    - Class filtering
    - Bounding box and label annotations
    """

    def __init__(
        self,
        state: SystemState,
        model_size: str = "n",  # n, s, m, l, x
        confidence_threshold: float = 0.30,
        classes: Optional[List[str]] = None,
        device: Optional[str] = None,
    ):
        """
        Initialize YOLO detector.

        Args:
            state: System state manager
            model_size: YOLO model size (n=nano, s=small, m=medium, l=large, x=xlarge)
            confidence_threshold: Minimum confidence for detections (0.0-1.0)
            classes: List of class names to detect (None = all COCO classes)
            device: Device to run inference on ('cuda', 'cpu', or None for auto)
        """
        self.state = state
        self.confidence_threshold = confidence_threshold
        self.target_classes = classes

        # Auto-select device
        if device is None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device

        logger.info(f"Initializing YOLOv11{model_size} detector on {self.device}")

        # Load YOLO model from weights directory
        import os
        from pathlib import Path

        weights_dir = Path("weights")
        weights_dir.mkdir(exist_ok=True)

        model_name = f"yolo11{model_size}.pt"
        model_path = weights_dir / model_name

        try:
            # If model exists in weights/, use it; otherwise YOLO will download to weights/
            if model_path.exists():
                self.model = YOLO(str(model_path))
            else:
                # Download to weights/ directory
                os.environ["YOLO_CONFIG_DIR"] = str(weights_dir)
                self.model = YOLO(model_name)
                # Move downloaded model to weights/ if it ended up elsewhere
                if Path(model_name).exists() and not model_path.exists():
                    Path(model_name).rename(model_path)
                    logger.info(f"Moved {model_name} to weights/ directory")

            self.model.to(self.device)
            logger.info(f"YOLOv11{model_size} model loaded successfully from {model_path}")
        except Exception as e:
            logger.error(f"Failed to load YOLO model: {e}")
            raise

        # COCO class names (YOLOv11 is trained on COCO dataset)
        self.class_names = self.model.names

        # Create class ID filter if specific classes requested
        if self.target_classes:
            self.class_ids = []
            for cls in self.target_classes:
                for class_id, class_name in self.class_names.items():
                    if class_name.lower() == cls.lower():
                        self.class_ids.append(class_id)
            logger.info(f"Filtering for classes: {self.target_classes} (IDs: {self.class_ids})")
        else:
            self.class_ids = None

        # Statistics
        self.frame_count = 0
        self.detection_count = 0

    def detect(
        self, frame: np.ndarray, timestamp: Optional[float] = None
    ) -> Tuple[np.ndarray, List[Detection]]:
        """
        Run object detection and tracking on a frame.

        Args:
            frame: Input image (BGR format)
            timestamp: Optional timestamp for the frame

        Returns:
            Tuple of (annotated_frame, detections)
        """
        self.frame_count += 1

        # Run inference with tracking
        # persist=True keeps the tracker state between frames
        # Use custom drone-optimized tracker with camera motion compensation
        results = self.model.track(
            frame,
            conf=self.confidence_threshold,
            classes=self.class_ids,
            verbose=False,
            persist=True,
            tracker="config/drone_tracker.yaml",  # BoTSORT with motion compensation
        )[
            0
        ]  # Get first result (single image)

        # Debug: Log tracker info on first few frames
        if self.frame_count <= 3:
            logger.info(f"Frame {self.frame_count}: Using tracker={results.speed.get('tracker', 'N/A')}ms, "
                       f"Detections={len(results.boxes) if results.boxes is not None else 0}")

        # Extract detections with track IDs
        detections = []
        boxes = results.boxes

        for box in boxes:
            # Get box coordinates (xyxy format)
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            # Get confidence and class
            confidence = float(box.conf[0].cpu().numpy())
            class_id = int(box.cls[0].cpu().numpy())
            class_name = self.class_names[class_id]

            # Get track ID if available
            track_id = None
            if box.id is not None:
                track_id = int(box.id[0].cpu().numpy())

            # Create detection object
            detection = Detection(
                class_name=class_name,
                confidence=confidence,
                bbox=(int(x1), int(y1), int(x2), int(y2)),
                timestamp=timestamp,
                track_id=track_id,
            )
            detections.append(detection)
            self.detection_count += 1

        # Update global state with detections
        self.state.update_detections(detections)

        # Create annotated frame
        annotated_frame = self._draw_detections(frame.copy(), detections)

        return annotated_frame, detections

    def _draw_detections(self, frame: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """
        Draw bounding boxes and labels on frame.

        Args:
            frame: Input frame
            detections: List of detections to draw

        Returns:
            Annotated frame
        """
        for detection in detections:
            x1, y1, x2, y2 = detection.bbox

            # Choose color based on class
            color = self._get_class_color(detection.class_name)

            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            # Draw label with just track ID (GPS shown in sidebar)
            if detection.track_id is not None:
                label = f"ID:{detection.track_id}"
            else:
                label = f"{detection.class_name}"

            label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

            # Draw label background
            cv2.rectangle(
                frame,
                (x1, y1 - label_size[1] - baseline - 5),
                (x1 + label_size[0], y1),
                color,
                -1,
            )

            # Draw label text
            cv2.putText(
                frame,
                label,
                (x1, y1 - baseline - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

        # Draw detection count
        cv2.putText(
            frame,
            f"Detections: {len(detections)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        return frame

    def _get_class_color(self, class_name: str) -> Tuple[int, int, int]:
        """
        Get consistent color for a class name.

        Args:
            class_name: Name of the class

        Returns:
            BGR color tuple
        """
        # Define colors for common classes
        class_colors = {
            "person": (0, 255, 0),  # Green
            "car": (255, 0, 0),  # Blue
            "truck": (0, 0, 255),  # Red
            "boat": (255, 255, 0),  # Cyan
            "airplane": (255, 0, 255),  # Magenta
            "bird": (0, 255, 255),  # Yellow
        }

        # Return predefined color or generate one
        if class_name in class_colors:
            return class_colors[class_name]
        else:
            # Generate consistent color from class name hash
            hash_val = hash(class_name)
            return (
                hash_val % 256,
                (hash_val // 256) % 256,
                (hash_val // 65536) % 256,
            )

    def get_stats(self) -> Dict[str, Any]:
        """
        Get detector statistics.

        Returns:
            Dictionary with statistics
        """
        return {
            "frames_processed": self.frame_count,
            "total_detections": self.detection_count,
            "avg_detections_per_frame": (
                self.detection_count / self.frame_count if self.frame_count > 0 else 0
            ),
            "device": self.device,
            "confidence_threshold": self.confidence_threshold,
            "target_classes": self.target_classes,
        }
