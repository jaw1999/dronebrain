"""
Streaming Module

Provides video streaming capabilities including:
- RTSP publishing to MediaMTX
- Raw video streams
- Annotated video streams with detections
"""

from src.streaming.stream_publisher import StreamPublisher

__all__ = ["StreamPublisher"]
