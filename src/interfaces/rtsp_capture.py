"""
RTSP Video Stream Capture Module.

This module handles capturing video frames from the Siyi A8 Mini camera
via RTSP protocol. It provides a thread-safe frame buffer for downstream
processing (vision models, streaming, etc.).

Features:
- Async frame capture in background thread
- Thread-safe frame buffer with latest frame access
- Automatic reconnection on stream failure
- Frame statistics (FPS, dropped frames)
- Hardware-accelerated decoding (when available)

Author: DroneBrain Team
"""

import cv2
import threading
import time
import logging
import os
from typing import Optional, Tuple
import numpy as np
from collections import deque

logger = logging.getLogger(__name__)


class RTSPCapture:
    """
    RTSP video stream capture with buffering.

    This class captures video frames from an RTSP stream in a background
    thread and provides thread-safe access to the latest frame.

    Attributes:
        rtsp_url: RTSP stream URL
        running: Thread running flag
        thread: Background capture thread
        latest_frame: Most recent captured frame
        frame_count: Total frames captured
        fps: Current frames per second

    Example:
        >>> capture = RTSPCapture("rtsp://192.168.144.25:8554/main.264")
        >>> capture.start()
        >>> frame = capture.get_frame()
        >>> if frame is not None:
        >>>     cv2.imshow("Camera", frame)
        >>> capture.stop()
    """

    def __init__(
        self,
        rtsp_url: str,
        buffer_size: int = 1,
        transport: str = "tcp",
        reconnect_delay: float = 5.0,
    ):
        """
        Initialize RTSP capture.

        Args:
            rtsp_url: RTSP stream URL (e.g., "rtsp://192.168.144.25:8554/main.264")
            buffer_size: Number of frames to buffer (1 = latest only)
            transport: Transport protocol ("tcp" or "udp")
            reconnect_delay: Seconds to wait before reconnection attempt
        """
        self.rtsp_url = rtsp_url
        self.buffer_size = buffer_size
        self.transport = transport
        self.reconnect_delay = reconnect_delay

        # Thread control
        self.running = False
        self.thread: Optional[threading.Thread] = None

        # Frame buffer (thread-safe)
        self._frame_lock = threading.Lock()
        self._frame_buffer = deque(maxlen=buffer_size)
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_timestamp = 0.0

        # Statistics
        self.frame_count = 0
        self.dropped_frames = 0
        self.fps = 0.0
        self._last_fps_update = time.time()
        self._fps_frame_count = 0

        # OpenCV capture object
        self._cap: Optional[cv2.VideoCapture] = None

        # Stream info
        self.width = 0
        self.height = 0
        self.codec = ""

    def start(self):
        """Start background capture thread."""
        if self.running:
            logger.warning("RTSP capture already running")
            return

        logger.info(f"Starting RTSP capture from {self.rtsp_url}")
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def stop(self):
        """Stop background capture thread and release resources."""
        if not self.running:
            return

        logger.info("Stopping RTSP capture...")
        self.running = False

        if self.thread:
            self.thread.join(timeout=5.0)

        self._release_capture()
        logger.info("RTSP capture stopped")

    def _init_capture(self) -> bool:
        """
        Initialize OpenCV video capture.

        Returns:
            True if successful
        """
        try:
            # Set environment variables for FFmpeg RTSP options
            if self.transport == "tcp":
                os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|stimeout;5000000"

            # Build RTSP URL with transport protocol
            url = self.rtsp_url

            logger.info(f"Initializing capture from {url} (transport: {self.transport})")

            # Create VideoCapture object with TCP transport
            self._cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)

            # Force TCP transport and reduce timeout
            if self.transport == "tcp":
                self._cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000)  # 5 second timeout
                self._cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000)

            # Set buffer size (reduce latency)
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            # Attempt to open
            if not self._cap.isOpened():
                logger.error("Failed to open RTSP stream")
                return False

            # Get stream properties
            self.width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = self._cap.get(cv2.CAP_PROP_FPS)

            logger.info(f"Stream opened: {self.width}x{self.height} @ {fps:.1f} FPS")

            return True

        except Exception as e:
            logger.error(f"Error initializing capture: {e}")
            return False

    def _release_capture(self):
        """Release OpenCV capture resources."""
        if self._cap:
            self._cap.release()
            self._cap = None

    def _capture_loop(self):
        """
        Main capture loop (runs in background thread).

        Continuously reads frames from RTSP stream and updates buffer.
        Automatically reconnects on stream failure.
        """
        logger.info("Capture loop started")

        while self.running:
            # Initialize or reconnect
            if not self._cap or not self._cap.isOpened():
                if not self._init_capture():
                    logger.warning(f"Failed to connect, retrying in {self.reconnect_delay}s...")
                    time.sleep(self.reconnect_delay)
                    continue

            try:
                # Read frame
                ret, frame = self._cap.read()

                if not ret or frame is None:
                    logger.warning("Failed to read frame, reconnecting...")
                    self._release_capture()
                    time.sleep(self.reconnect_delay)
                    continue

                # Update frame buffer (thread-safe)
                with self._frame_lock:
                    self._frame_buffer.append(frame.copy())
                    self._latest_frame = frame
                    self._frame_timestamp = time.time()

                # Update statistics
                self.frame_count += 1
                self._fps_frame_count += 1

                # Calculate FPS every second
                now = time.time()
                if now - self._last_fps_update >= 1.0:
                    self.fps = self._fps_frame_count / (now - self._last_fps_update)
                    self._fps_frame_count = 0
                    self._last_fps_update = now

            except Exception as e:
                logger.error(f"Error in capture loop: {e}")
                self._release_capture()
                time.sleep(self.reconnect_delay)

        # Cleanup
        self._release_capture()
        logger.info("Capture loop stopped")

    # ===== Public Frame Access Methods =====

    def get_frame(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        """
        Get the latest captured frame.

        Args:
            timeout: Maximum time to wait for a frame (seconds)

        Returns:
            Frame as numpy array (BGR format) or None if unavailable
        """
        start_time = time.time()

        while time.time() - start_time < timeout:
            with self._frame_lock:
                if self._latest_frame is not None:
                    return self._latest_frame.copy()

            time.sleep(0.01)  # Small delay to avoid busy-waiting

        return None

    def get_frame_with_timestamp(self, timeout: float = 1.0) -> Optional[Tuple[np.ndarray, float]]:
        """
        Get latest frame with capture timestamp.

        Args:
            timeout: Maximum time to wait for a frame (seconds)

        Returns:
            Tuple of (frame, timestamp) or None if unavailable
        """
        start_time = time.time()

        while time.time() - start_time < timeout:
            with self._frame_lock:
                if self._latest_frame is not None:
                    return (self._latest_frame.copy(), self._frame_timestamp)

            time.sleep(0.01)

        return None

    def get_buffered_frames(self) -> list:
        """
        Get all frames in buffer.

        Returns:
            List of frames (oldest to newest)
        """
        with self._frame_lock:
            return list(self._frame_buffer)

    def is_opened(self) -> bool:
        """
        Check if stream is currently open.

        Returns:
            True if stream is open and receiving frames
        """
        if not self._cap:
            return False

        with self._frame_lock:
            # Check if we received a frame recently (within last 2 seconds)
            if self._frame_timestamp == 0:
                return False

            return (time.time() - self._frame_timestamp) < 2.0

    def get_resolution(self) -> Tuple[int, int]:
        """
        Get stream resolution.

        Returns:
            Tuple of (width, height)
        """
        return (self.width, self.height)

    def get_fps(self) -> float:
        """
        Get current frames per second.

        Returns:
            Current FPS
        """
        return self.fps

    def get_stats(self) -> dict:
        """
        Get capture statistics.

        Returns:
            Dict with statistics
        """
        with self._frame_lock:
            age = time.time() - self._frame_timestamp if self._frame_timestamp > 0 else 0

        return {
            "running": self.running,
            "opened": self.is_opened(),
            "resolution": f"{self.width}x{self.height}",
            "fps": round(self.fps, 2),
            "total_frames": self.frame_count,
            "dropped_frames": self.dropped_frames,
            "latest_frame_age": round(age, 3),
        }


class RTSPCaptureGStreamer(RTSPCapture):
    """
    RTSP capture using GStreamer backend for better performance.

    This variant uses GStreamer for hardware-accelerated decoding on
    NVIDIA Jetson platforms. Falls back to FFmpeg if GStreamer unavailable.

    Example:
        >>> # On Jetson with GStreamer
        >>> capture = RTSPCaptureGStreamer("rtsp://192.168.144.25:8554/main.264")
        >>> capture.start()
    """

    def _init_capture(self) -> bool:
        """
        Initialize capture using GStreamer pipeline.

        Returns:
            True if successful
        """
        try:
            # GStreamer pipeline for hardware-accelerated decoding
            # Uses nvv4l2decoder on Jetson for H.264/H.265 decoding
            pipeline = (
                f"rtspsrc location={self.rtsp_url} protocols=tcp latency=0 ! "
                "rtph264depay ! h264parse ! "
                "nvv4l2decoder ! nvvidconv ! "
                "video/x-raw,format=BGRx ! "
                "videoconvert ! video/x-raw,format=BGR ! "
                "appsink drop=1 sync=0"
            )

            logger.info("Initializing GStreamer capture")
            logger.debug(f"Pipeline: {pipeline}")

            self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

            if not self._cap.isOpened():
                logger.warning("GStreamer failed, falling back to FFmpeg")
                return super()._init_capture()

            # Get stream properties
            self.width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            logger.info(f"GStreamer stream opened: {self.width}x{self.height}")

            return True

        except Exception as e:
            logger.error(f"GStreamer error: {e}, falling back to FFmpeg")
            return super()._init_capture()
