"""
Target Tracker Module

Implements autonomous tracking of detected objects using gimbal control.
Tracks targets by keeping them centered in the camera frame.
"""

import logging
import time
from enum import Enum
from typing import Optional, List
from dataclasses import dataclass

from src.control.pid_controller import PIDController
from src.vision.yolo_detector import Detection

logger = logging.getLogger(__name__)


class TrackingState(Enum):
    """Tracking state machine states."""

    IDLE = "IDLE"  # No active tracking
    LOCKED = "LOCKED"  # Target acquired and locked
    TRACKING = "TRACKING"  # Actively tracking target
    LOST = "LOST"  # Target lost, attempting recovery


@dataclass
class TrackedTarget:
    """Information about currently tracked target."""

    track_id: int
    class_name: str
    last_seen_time: float
    center_x: float
    center_y: float
    bbox: tuple[int, int, int, int]
    confidence: float
    frames_tracked: int = 0
    frames_lost: int = 0


class TargetTracker:
    """
    Autonomous target tracking using gimbal control.

    Features:
    - PID-based smooth gimbal control
    - Target lock and persistence
    - Lost target recovery
    - Configurable tracking parameters

    Example:
        >>> tracker = TargetTracker(
        ...     camera_width=1280,
        ...     camera_height=720,
        ...     pid_yaw_gains=(1.0, 0.1, 0.05),
        ...     pid_pitch_gains=(1.0, 0.1, 0.05)
        ... )
        >>> tracker.lock_target(track_id=5)
        >>> yaw_cmd, pitch_cmd = tracker.update(detections, dt=0.033)
    """

    def __init__(
        self,
        camera_width: int = 1280,
        camera_height: int = 720,
        pid_yaw_gains: tuple[float, float, float] = (1.5, 0.0, 0.1),
        pid_pitch_gains: tuple[float, float, float] = (1.5, 0.0, 0.1),
        deadzone_px: int = 30,
        max_gimbal_speed: float = 30.0,
        lost_timeout: float = 2.0,
    ):
        """
        Initialize target tracker.

        Args:
            camera_width: Camera frame width in pixels
            camera_height: Camera frame height in pixels
            pid_yaw_gains: PID gains for yaw control (kp, ki, kd)
            pid_pitch_gains: PID gains for pitch control (kp, ki, kd)
            deadzone_px: Deadzone radius in pixels (no tracking if within this)
            max_gimbal_speed: Maximum gimbal speed in degrees/sec
            lost_timeout: Time in seconds before giving up on lost target
        """
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.center_x = camera_width / 2
        self.center_y = camera_height / 2
        self.deadzone_px = deadzone_px
        self.max_gimbal_speed = max_gimbal_speed
        self.lost_timeout = lost_timeout

        # PID controllers for yaw and pitch
        self.pid_yaw = PIDController(
            kp=pid_yaw_gains[0],
            ki=pid_yaw_gains[1],
            kd=pid_yaw_gains[2],
            output_limits=(-max_gimbal_speed, max_gimbal_speed),
            integral_limits=(-10.0, 10.0),
        )

        self.pid_pitch = PIDController(
            kp=pid_pitch_gains[0],
            ki=pid_pitch_gains[1],
            kd=pid_pitch_gains[2],
            output_limits=(-max_gimbal_speed, max_gimbal_speed),
            integral_limits=(-10.0, 10.0),
        )

        # Tracking state
        self.state = TrackingState.IDLE
        self.target: Optional[TrackedTarget] = None
        self.locked_track_id: Optional[int] = None

    def lock_target(self, track_id: int) -> bool:
        """
        Lock onto a specific track ID for tracking.

        Args:
            track_id: Track ID to follow

        Returns:
            True if lock successful
        """
        self.locked_track_id = track_id
        self.state = TrackingState.LOCKED
        self.pid_yaw.reset()
        self.pid_pitch.reset()
        logger.info(f"Locked onto target: Track ID {track_id}")
        return True

    def unlock_target(self):
        """Release current target and stop tracking."""
        if self.target:
            logger.info(f"Unlocked target: Track ID {self.target.track_id}")
        self.locked_track_id = None
        self.target = None
        self.state = TrackingState.IDLE
        self.pid_yaw.reset()
        self.pid_pitch.reset()

    def update(
        self, detections: List[Detection], dt: Optional[float] = None
    ) -> tuple[float, float]:
        """
        Update tracker with new detections and calculate gimbal commands.

        Args:
            detections: List of current detections
            dt: Time delta since last update (seconds)

        Returns:
            Tuple of (yaw_rate, pitch_rate) in degrees/sec
        """
        current_time = time.time()

        # If no locked target, do nothing
        if self.locked_track_id is None:
            return 0.0, 0.0

        # Find the locked target in current detections
        target_detection = None
        for detection in detections:
            if detection.track_id == self.locked_track_id:
                target_detection = detection
                break

        # Update target state
        if target_detection:
            # Target found - update tracking info
            x1, y1, x2, y2 = target_detection.bbox
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2

            if self.target:
                # Update existing target
                self.target.center_x = center_x
                self.target.center_y = center_y
                self.target.bbox = target_detection.bbox
                self.target.confidence = target_detection.confidence
                self.target.last_seen_time = current_time
                self.target.frames_tracked += 1
                self.target.frames_lost = 0
            else:
                # Create new target
                self.target = TrackedTarget(
                    track_id=self.locked_track_id,
                    class_name=target_detection.class_name,
                    last_seen_time=current_time,
                    center_x=center_x,
                    center_y=center_y,
                    bbox=target_detection.bbox,
                    confidence=target_detection.confidence,
                    frames_tracked=1,
                    frames_lost=0,
                )

            # Update state to TRACKING
            if self.state != TrackingState.TRACKING:
                logger.info(
                    f"Started tracking: Track ID {self.locked_track_id} "
                    f"({target_detection.class_name})"
                )
                self.state = TrackingState.TRACKING

        else:
            # Target not found in current frame
            if self.target:
                self.target.frames_lost += 1

                # Check if target has been lost too long
                time_lost = current_time - self.target.last_seen_time
                if time_lost > self.lost_timeout:
                    logger.warning(
                        f"Target lost for {time_lost:.1f}s, giving up: "
                        f"Track ID {self.locked_track_id}"
                    )
                    self.unlock_target()
                    return 0.0, 0.0

                # Update state to LOST
                if self.state == TrackingState.TRACKING:
                    logger.warning(f"Target lost: Track ID {self.locked_track_id}")
                    self.state = TrackingState.LOST

        # Calculate gimbal commands if we have a target
        if self.target and self.state in (TrackingState.TRACKING, TrackingState.LOST):
            # Calculate error (distance from center)
            error_x = self.target.center_x - self.center_x
            error_y = self.target.center_y - self.center_y

            # Apply deadzone - don't track if target is close enough to center
            distance_from_center = (error_x**2 + error_y**2) ** 0.5
            if distance_from_center < self.deadzone_px:
                return 0.0, 0.0

            # Convert pixel error to normalized error (-1 to 1)
            # Yaw: positive error means target is to the right, need to pan right
            # Pitch: positive error means target is down, need to tilt down
            # BUT: gimbal pitch command is inverted (negative = down)
            norm_error_yaw = error_x / (self.camera_width / 2)
            norm_error_pitch = error_y / (self.camera_height / 2)

            # Calculate PID outputs (degrees/sec)
            yaw_rate = self.pid_yaw.update(norm_error_yaw, dt=dt)
            pitch_rate = -self.pid_pitch.update(norm_error_pitch, dt=dt)  # Inverted for gimbal

            return yaw_rate, pitch_rate

        return 0.0, 0.0

    def get_state(self) -> TrackingState:
        """Get current tracking state."""
        return self.state

    def get_target(self) -> Optional[TrackedTarget]:
        """Get currently tracked target info."""
        return self.target

    def is_tracking(self) -> bool:
        """Check if actively tracking a target."""
        return self.state in (TrackingState.TRACKING, TrackingState.LOST)
