"""
Vision Module

Provides computer vision capabilities for DroneBrain including:
- Object detection (YOLOv11, future: Grounding DINO)
- Instance segmentation (future: SAM)
- Tracking (ByteTrack via YOLO)
- Geolocation (photogrammetry)
- Annotation
"""

from src.vision.yolo_detector import YOLODetector

__all__ = ["YOLODetector"]
