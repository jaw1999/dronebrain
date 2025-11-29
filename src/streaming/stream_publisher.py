"""
Video Stream Publisher Module

Publishes video streams to MediaMTX server via RTSP.
Supports both raw and annotated video streams.
"""

import logging
import subprocess
import threading
import time
from typing import Optional
import numpy as np
import cv2

logger = logging.getLogger(__name__)


class StreamPublisher:
    """
    Publishes video frames to MediaMTX via FFmpeg RTSP stream.

    Features:
    - Raw video streaming (direct camera passthrough)
    - Annotated video streaming (with detection overlays)
    - Automatic reconnection on failure
    - Thread-safe frame publishing

    Example:
        >>> publisher = StreamPublisher(
        ...     stream_name="annotated",
        ...     width=1280,
        ...     height=720,
        ...     fps=25
        ... )
        >>> publisher.start()
        >>> publisher.publish_frame(frame)
        >>> publisher.stop()
    """

    def __init__(
        self,
        stream_name: str = "annotated",
        mediamtx_host: str = "localhost",
        mediamtx_port: int = 8554,
        width: int = 1280,
        height: int = 720,
        fps: int = 25,
        bitrate: str = "2M",
    ):
        """
        Initialize stream publisher.

        Args:
            stream_name: Name of the stream path on MediaMTX
            mediamtx_host: MediaMTX server hostname/IP
            mediamtx_port: MediaMTX RTSP port
            width: Video width in pixels
            height: Video height in pixels
            fps: Frames per second
            bitrate: Video bitrate (e.g., "2M", "4M")
        """
        self.stream_name = stream_name
        self.mediamtx_url = f"rtsp://{mediamtx_host}:{mediamtx_port}/{stream_name}"
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = bitrate

        self.process: Optional[subprocess.Popen] = None
        self.running = False
        self._lock = threading.Lock()

        # Statistics
        self.frames_published = 0
        self.last_publish_time = 0.0
        self.errors = 0

    def start(self) -> bool:
        """
        Start the FFmpeg process for streaming.

        Returns:
            True if started successfully
        """
        if self.running:
            logger.warning(f"Stream publisher '{self.stream_name}' already running")
            return False

        try:
            # FFmpeg command to publish RTSP stream to MediaMTX
            # Uses H.264 codec with low latency settings
            command = [
                "ffmpeg",
                "-y",  # Overwrite output
                "-f",
                "rawvideo",  # Input format
                "-vcodec",
                "rawvideo",
                "-pix_fmt",
                "bgr24",  # OpenCV uses BGR
                "-s",
                f"{self.width}x{self.height}",
                "-r",
                str(self.fps),
                "-i",
                "-",  # Read from stdin
                "-c:v",
                "libx264",  # H.264 codec
                "-preset",
                "ultrafast",  # Low latency
                "-tune",
                "zerolatency",
                "-b:v",
                self.bitrate,
                "-maxrate",
                self.bitrate,
                "-bufsize",
                f"{float(self.bitrate[:-1]) * 2:.1f}M",
                "-pix_fmt",
                "yuv420p",
                "-g",
                str(self.fps * 2),  # GOP size
                "-f",
                "rtsp",  # Output format
                "-rtsp_transport",
                "tcp",
                self.mediamtx_url,
            ]

            logger.info(f"Starting stream publisher: {self.stream_name}")
            logger.debug(f"FFmpeg command: {' '.join(command)}")

            self.process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            self.running = True
            self.frames_published = 0
            self.errors = 0

            logger.info(f"✓ Stream publisher started: rtsp://localhost:8554/{self.stream_name}")
            return True

        except Exception as e:
            logger.error(f"Failed to start stream publisher: {e}")
            return False

    def publish_frame(self, frame: np.ndarray) -> bool:
        """
        Publish a single frame to the stream.

        Args:
            frame: Video frame (BGR format)

        Returns:
            True if published successfully
        """
        if not self.running or self.process is None:
            return False

        # Check if FFmpeg process is still alive
        if self.process.poll() is not None:
            # Process died - try to get error output
            try:
                stderr_output = self.process.stderr.read().decode('utf-8', errors='ignore')
                if stderr_output:
                    logger.error(f"FFmpeg error for {self.stream_name}: {stderr_output[-500:]}")  # Last 500 chars
                else:
                    logger.error(f"FFmpeg process died for stream: {self.stream_name} (no error output)")
            except Exception as e:
                logger.error(f"FFmpeg process died for stream: {self.stream_name} (couldn't read error: {e})")

            self.running = False
            return False

        try:
            with self._lock:
                # Resize frame if needed
                if frame.shape[:2] != (self.height, self.width):
                    frame = cv2.resize(frame, (self.width, self.height))

                # Write frame to FFmpeg stdin
                self.process.stdin.write(frame.tobytes())
                self.process.stdin.flush()

                self.frames_published += 1
                self.last_publish_time = time.time()

                return True

        except BrokenPipeError:
            logger.error(f"Stream publisher pipe broken: {self.stream_name}")
            self.errors += 1
            self.running = False
            return False
        except Exception as e:
            logger.error(f"Error publishing frame: {e}")
            self.errors += 1
            return False

    def stop(self):
        """Stop the stream publisher and cleanup resources."""
        if not self.running:
            return

        logger.info(f"Stopping stream publisher: {self.stream_name}")

        self.running = False

        if self.process:
            try:
                # Close stdin to signal FFmpeg to terminate
                if self.process.stdin:
                    self.process.stdin.close()

                # Wait for process to terminate
                self.process.wait(timeout=2.0)

            except subprocess.TimeoutExpired:
                logger.warning("FFmpeg didn't terminate gracefully, killing it")
                self.process.kill()
                self.process.wait()
            except Exception as e:
                logger.error(f"Error stopping stream publisher: {e}")
            finally:
                self.process = None

        logger.info(f"Stream publisher stopped: {self.stream_name}")
        logger.info(f"  Total frames published: {self.frames_published}")

    def is_running(self) -> bool:
        """
        Check if stream publisher is running.

        Returns:
            True if running
        """
        return self.running and self.process is not None

    def get_stats(self) -> dict:
        """
        Get publisher statistics.

        Returns:
            Dictionary with stats
        """
        return {
            "stream_name": self.stream_name,
            "running": self.running,
            "frames_published": self.frames_published,
            "errors": self.errors,
            "url": self.mediamtx_url,
        }
