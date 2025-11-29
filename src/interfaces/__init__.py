"""
Hardware interface modules for DroneBrain.

This package contains interfaces for:
- MAVLink communication with flight controller
- Siyi gimbal control
- RTSP video stream capture
"""

from .mavlink_interface import MAVLinkInterface
from .siyi_interface import SiyiInterface
from .rtsp_capture import RTSPCapture

__all__ = ["MAVLinkInterface", "SiyiInterface", "RTSPCapture"]
