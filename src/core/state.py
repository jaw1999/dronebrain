"""
Global system state management for DroneBrain.

This module provides a centralized state manager that holds all system data
including telemetry, detections, tracking state, and mission status.
Thread-safe access is provided for all components.

"""

import threading
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any
from datetime import datetime
from enum import Enum


class MissionMode(Enum):
    """Current autonomous mission mode"""

    MANUAL = "manual"
    LOITER = "loiter"
    FOLLOW = "follow"
    RTL = "rtl"  # Return to launch


class SystemStatus(Enum):
    """Overall system status"""

    INITIALIZING = "initializing"
    READY = "ready"
    ACTIVE = "active"
    ERROR = "error"
    SHUTDOWN = "shutdown"


@dataclass
class Detection:
    """
    Object detection result with optional geolocation.

    Attributes:
        class_name: Detected class label
        confidence: Detection confidence (0.0-1.0)
        bbox: Bounding box coordinates (x1, y1, x2, y2)
        timestamp: Detection timestamp
        track_id: Optional tracking ID
        latitude: Optional GPS latitude (degrees)
        longitude: Optional GPS longitude (degrees)
        distance: Optional slant range distance (meters)
    """

    class_name: str
    confidence: float
    bbox: tuple  # (x1, y1, x2, y2)
    timestamp: Optional[float] = None
    track_id: Optional[int] = None
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    distance: Optional[float] = None


@dataclass
class DroneState:
    """
    Current drone telemetry and status.

    Attributes:
        timestamp: Time of last update
        connected: MAVLink connection status
        armed: Motor armed status
        mode: Current flight mode (from autopilot)
        latitude: GPS latitude (degrees)
        longitude: GPS longitude (degrees)
        altitude: Altitude MSL (meters)
        altitude_relative: Altitude AGL (meters)
        heading: Compass heading (degrees, 0-360)
        groundspeed: Ground speed (m/s)
        airspeed: Air speed (m/s)
        climb_rate: Vertical speed (m/s)
        battery_voltage: Battery voltage (volts)
        battery_current: Battery current (amps)
        battery_remaining: Battery percentage (0-100)
        gps_fix: GPS fix type (0=no fix, 3=3D fix)
        gps_satellites: Number of satellites
        roll: Roll angle (degrees)
        pitch: Pitch angle (degrees)
        yaw: Yaw angle (degrees)
    """

    timestamp: datetime = field(default_factory=datetime.now)
    connected: bool = False
    armed: bool = False
    mode: str = "UNKNOWN"

    # Position
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    altitude_relative: float = 0.0
    heading: float = 0.0

    # Velocity
    groundspeed: float = 0.0
    airspeed: float = 0.0
    climb_rate: float = 0.0

    # Battery
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    battery_remaining: float = 100.0

    # GPS
    gps_fix: int = 0
    gps_satellites: int = 0

    # Attitude
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class GimbalState:
    """
    Current gimbal telemetry and status.

    Attributes:
        timestamp: Time of last update
        connected: Gimbal connection status
        yaw: Gimbal yaw angle (degrees, -135 to 135)
        pitch: Gimbal pitch angle (degrees, -90 to 25)
        roll: Gimbal roll angle (degrees)
        locked_target_id: ID of locked target (None if not locked)
    """

    timestamp: datetime = field(default_factory=datetime.now)
    connected: bool = False
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    locked_target_id: Optional[str] = None


@dataclass
class CameraState:
    """
    Current camera status and statistics.

    Attributes:
        timestamp: Time of last update
        connected: Camera connection status
        resolution: Current resolution (e.g., "1920x1080")
        fps: Current frames per second
        frame_count: Total frames processed
    """

    timestamp: datetime = field(default_factory=datetime.now)
    connected: bool = False
    resolution: str = "Unknown"
    fps: float = 0.0
    frame_count: int = 0


@dataclass
class DetectedObject:
    """
    A detected object from the vision system.

    Attributes:
        id: Unique identifier for this detection
        timestamp: Time of detection
        class_name: Object class (e.g., "person", "vehicle")
        confidence: Detection confidence (0.0 to 1.0)
        bbox: Bounding box in image coordinates [x1, y1, x2, y2]
        center: Center point in image coordinates [x, y]
        mask: Segmentation mask (optional, from SAM)

        # Geolocation (calculated via photogrammetry)
        latitude: Calculated GPS latitude
        longitude: Calculated GPS longitude
        altitude: Assumed altitude (typically 0 for ground targets)

        # Tracking
        tracked: Whether this object is being tracked
        track_id: Unique tracking ID (persistent across frames)
    """

    id: str
    timestamp: datetime = field(default_factory=datetime.now)
    class_name: str = "unknown"
    confidence: float = 0.0
    bbox: List[float] = field(default_factory=list)  # [x1, y1, x2, y2]
    center: List[float] = field(default_factory=list)  # [x, y]
    mask: Optional[Any] = None  # numpy array

    # Geolocation
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude: Optional[float] = None
    distance: Optional[float] = None  # Slant range distance in meters

    # Tracking
    tracked: bool = False
    track_id: Optional[int] = None


@dataclass
class MissionState:
    """
    Current autonomous mission status.

    Attributes:
        mode: Current mission mode
        active: Whether mission is actively running
        target_id: ID of target being tracked/followed
        waypoints: List of mission waypoints
        current_waypoint: Index of current waypoint
    """

    mode: MissionMode = MissionMode.MANUAL
    active: bool = False
    target_id: Optional[str] = None
    waypoints: List[Dict[str, Any]] = field(default_factory=list)
    current_waypoint: int = 0


class SystemState:
    """
    Thread-safe global state manager for DroneBrain.

    This class maintains all system state including:
    - Drone telemetry
    - Gimbal status
    - Vision detections
    - Mission state
    - System status

    All access is protected by locks for thread safety.

    Example:
        >>> state = SystemState()
        >>> state.update_drone_position(lat=37.7749, lon=-122.4194, alt=100)
        >>> drone_state = state.get_drone_state()
        >>> print(f"Drone at {drone_state.latitude}, {drone_state.longitude}")
    """

    def __init__(self):
        """Initialize system state with default values."""
        self._lock = threading.RLock()

        # Component states
        self._drone = DroneState()
        self._gimbal = GimbalState()
        self._camera = CameraState()
        self._mission = MissionState()

        # Detections (keyed by detection ID)
        self._detections: Dict[str, DetectedObject] = {}

        # Frame processing stats
        self._last_frame_time = None
        self._frame_times = []  # For FPS calculation

        # System status
        self._status = SystemStatus.INITIALIZING
        self._errors: List[str] = []

        # Configuration (loaded from config file)
        self._config: Dict[str, Any] = {}

    # ===== Drone State Methods =====

    def update_drone_state(self, **kwargs):
        """
        Update drone state fields.

        Args:
            **kwargs: Fields to update (e.g., latitude=37.7, altitude=100)
        """
        with self._lock:
            for key, value in kwargs.items():
                if hasattr(self._drone, key):
                    setattr(self._drone, key, value)
            self._drone.timestamp = datetime.now()

    def get_drone_state(self) -> DroneState:
        """
        Get current drone state (thread-safe copy).

        Returns:
            DroneState: Current drone telemetry
        """
        with self._lock:
            # Return a copy to prevent external modification
            return DroneState(**self._drone.__dict__)

    def update_drone_position(self, lat: float, lon: float, alt: float, alt_rel: float = None):
        """
        Update drone GPS position.

        Args:
            lat: Latitude (degrees)
            lon: Longitude (degrees)
            alt: Altitude MSL (meters)
            alt_rel: Altitude AGL (meters), optional
        """
        with self._lock:
            self._drone.latitude = lat
            self._drone.longitude = lon
            self._drone.altitude = alt
            if alt_rel is not None:
                self._drone.altitude_relative = alt_rel
            self._drone.timestamp = datetime.now()

    def update_drone_attitude(self, roll: float, pitch: float, yaw: float):
        """
        Update drone attitude.

        Args:
            roll: Roll angle (degrees)
            pitch: Pitch angle (degrees)
            yaw: Yaw angle (degrees)
        """
        with self._lock:
            self._drone.roll = roll
            self._drone.pitch = pitch
            self._drone.yaw = yaw
            self._drone.timestamp = datetime.now()

    # ===== Gimbal State Methods =====

    def update_gimbal_state(self, **kwargs):
        """
        Update gimbal state fields.

        Args:
            **kwargs: Fields to update (e.g., yaw=30, pitch=-20)
        """
        with self._lock:
            for key, value in kwargs.items():
                if hasattr(self._gimbal, key):
                    setattr(self._gimbal, key, value)
            self._gimbal.timestamp = datetime.now()

    def get_gimbal_state(self) -> GimbalState:
        """
        Get current gimbal state (thread-safe copy).

        Returns:
            GimbalState: Current gimbal telemetry
        """
        with self._lock:
            return GimbalState(**self._gimbal.__dict__)

    def set_gimbal_lock(self, target_id: Optional[str]):
        """
        Set or clear gimbal target lock.

        Args:
            target_id: ID of target to lock, or None to unlock
        """
        with self._lock:
            self._gimbal.locked_target_id = target_id

    def _calculate_vx(self) -> float:
        """
        Calculate drone velocity in X direction (East/West) from groundspeed and heading.
        Must be called within a lock context.

        Returns:
            float: Velocity in m/s (positive = East, negative = West)
        """
        if self._drone.groundspeed is None or self._drone.yaw is None:
            return 0.0
        # Convert heading from degrees to radians
        # Heading is degrees from North, so vx = groundspeed * sin(heading)
        heading_rad = math.radians(self._drone.yaw)
        return self._drone.groundspeed * math.sin(heading_rad)

    def _calculate_vy(self) -> float:
        """
        Calculate drone velocity in Y direction (North/South) from groundspeed and heading.
        Must be called within a lock context.

        Returns:
            float: Velocity in m/s (positive = North, negative = South)
        """
        if self._drone.groundspeed is None or self._drone.yaw is None:
            return 0.0
        # Convert heading from degrees to radians
        # Heading is degrees from North, so vy = groundspeed * cos(heading)
        heading_rad = math.radians(self._drone.yaw)
        return self._drone.groundspeed * math.cos(heading_rad)

    # ===== Camera State Methods =====

    def update_camera_state(self, **kwargs):
        """
        Update camera state fields.

        Args:
            **kwargs: Fields to update (e.g., fps=30, resolution="1920x1080")
        """
        with self._lock:
            for key, value in kwargs.items():
                if hasattr(self._camera, key):
                    setattr(self._camera, key, value)
            self._camera.timestamp = datetime.now()

    def get_camera_state(self) -> CameraState:
        """
        Get current camera state (thread-safe copy).

        Returns:
            CameraState: Current camera status
        """
        with self._lock:
            return CameraState(**self._camera.__dict__)

    def increment_frame_count(self):
        """Increment frame counter and update FPS estimate."""
        with self._lock:
            self._camera.frame_count += 1
            now = time.time()

            # Calculate FPS from recent frame times
            if self._last_frame_time is not None:
                self._frame_times.append(now - self._last_frame_time)
                # Keep only last 30 frames for averaging
                if len(self._frame_times) > 30:
                    self._frame_times.pop(0)
                # Calculate average FPS
                if self._frame_times:
                    avg_frame_time = sum(self._frame_times) / len(self._frame_times)
                    self._camera.fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0.0

            self._last_frame_time = now
            self._camera.timestamp = datetime.now()

    # ===== Detection Methods =====

    def add_detection(self, detection: DetectedObject):
        """
        Add a new detection to the state.

        Args:
            detection: DetectedObject instance
        """
        with self._lock:
            self._detections[detection.id] = detection

    def update_detection(self, detection_id: str, **kwargs):
        """
        Update fields of an existing detection.

        Args:
            detection_id: ID of detection to update
            **kwargs: Fields to update
        """
        with self._lock:
            if detection_id in self._detections:
                detection = self._detections[detection_id]
                for key, value in kwargs.items():
                    if hasattr(detection, key):
                        setattr(detection, key, value)

    def get_detection(self, detection_id: str) -> Optional[DetectedObject]:
        """
        Get a specific detection by ID.

        Args:
            detection_id: ID of detection

        Returns:
            DetectedObject or None if not found
        """
        with self._lock:
            return self._detections.get(detection_id)

    def get_all_detections(self) -> List[DetectedObject]:
        """
        Get all current detections.

        Returns:
            List of DetectedObject instances
        """
        with self._lock:
            return list(self._detections.values())

    def clear_old_detections(self, max_age_seconds: float = 5.0):
        """
        Remove detections older than specified age.

        Args:
            max_age_seconds: Maximum age to keep (default: 5 seconds)
        """
        with self._lock:
            now = datetime.now()
            to_remove = []
            for det_id, detection in self._detections.items():
                age = (now - detection.timestamp).total_seconds()
                if age > max_age_seconds:
                    to_remove.append(det_id)

            for det_id in to_remove:
                del self._detections[det_id]

    def update_detections(self, detections: List["Detection"]):
        """
        Update state with a new list of detections from vision processing.

        This method converts lightweight Detection objects (from YOLO) into
        DetectedObject instances and stores them in the state. Old detections
        are automatically cleared.

        Args:
            detections: List of Detection objects from vision detector
        """
        with self._lock:
            # Clear old detections first
            self.clear_old_detections(max_age_seconds=1.0)

            # Add new detections
            for detection in detections:
                # Generate unique ID for this detection
                detection_id = (
                    f"det_{int(detection.timestamp * 1000)}_"
                    f"{detection.class_name}_{id(detection)}"
                )

                # Calculate center point from bounding box
                x1, y1, x2, y2 = detection.bbox
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                # Create DetectedObject from Detection
                detected_obj = DetectedObject(
                    id=detection_id,
                    timestamp=(
                        datetime.fromtimestamp(detection.timestamp)
                        if detection.timestamp
                        else datetime.now()
                    ),
                    class_name=detection.class_name,
                    confidence=detection.confidence,
                    bbox=[float(x1), float(y1), float(x2), float(y2)],
                    center=[float(center_x), float(center_y)],
                    track_id=detection.track_id,
                    # Copy GPS/geolocation data if available
                    latitude=getattr(detection, "latitude", None),
                    longitude=getattr(detection, "longitude", None),
                    distance=getattr(detection, "distance", None),
                )

                # Store in detections dict
                self._detections[detection_id] = detected_obj

    # ===== Mission State Methods =====

    def set_mission_mode(self, mode: MissionMode):
        """
        Set current mission mode.

        Args:
            mode: MissionMode enum value
        """
        with self._lock:
            self._mission.mode = mode

    def get_mission_mode(self) -> MissionMode:
        """
        Get current mission mode.

        Returns:
            MissionMode enum value
        """
        with self._lock:
            return self._mission.mode

    def set_mission_active(self, active: bool):
        """
        Set mission active status.

        Args:
            active: True to activate mission, False to deactivate
        """
        with self._lock:
            self._mission.active = active

    def is_mission_active(self) -> bool:
        """
        Check if mission is currently active.

        Returns:
            True if mission is active
        """
        with self._lock:
            return self._mission.active

    def set_mission_target(self, target_id: Optional[str]):
        """
        Set mission target.

        Args:
            target_id: ID of target, or None to clear
        """
        with self._lock:
            self._mission.target_id = target_id

    def get_mission_state(self) -> MissionState:
        """
        Get current mission state (thread-safe copy).

        Returns:
            MissionState instance
        """
        with self._lock:
            return MissionState(**self._mission.__dict__)

    # ===== System Status Methods =====

    def set_status(self, status: SystemStatus):
        """
        Set overall system status.

        Args:
            status: SystemStatus enum value
        """
        with self._lock:
            self._status = status

    def get_status(self) -> SystemStatus:
        """
        Get current system status.

        Returns:
            SystemStatus enum value
        """
        with self._lock:
            return self._status

    def add_error(self, error_message: str):
        """
        Add an error message to the error log.

        Args:
            error_message: Error description
        """
        with self._lock:
            self._errors.append(f"[{datetime.now()}] {error_message}")
            # Keep only last 100 errors
            if len(self._errors) > 100:
                self._errors = self._errors[-100:]

    def get_errors(self) -> List[str]:
        """
        Get recent error messages.

        Returns:
            List of error strings
        """
        with self._lock:
            return self._errors.copy()

    def clear_errors(self):
        """Clear all error messages."""
        with self._lock:
            self._errors.clear()

    # ===== Configuration Methods =====

    def set_config(self, config: Dict[str, Any]):
        """
        Set system configuration.

        Args:
            config: Configuration dictionary
        """
        with self._lock:
            self._config = config

    def get_config(self) -> Dict[str, Any]:
        """
        Get system configuration.

        Returns:
            Configuration dictionary
        """
        with self._lock:
            return self._config.copy()

    def get_config_value(self, key_path: str, default: Any = None) -> Any:
        """
        Get a specific configuration value using dot notation.

        Args:
            key_path: Dot-separated path (e.g., "drone.mavlink_ip")
            default: Default value if key not found

        Returns:
            Configuration value or default

        Example:
            >>> state.get_config_value("camera.siyi_ip", "192.168.144.25")
        """
        with self._lock:
            keys = key_path.split(".")
            value = self._config
            for key in keys:
                if isinstance(value, dict) and key in value:
                    value = value[key]
                else:
                    return default
            return value

    # ===== Utility Methods =====

    def get_summary(self) -> Dict[str, Any]:
        """
        Get a summary of all system state.

        Returns:
            Dictionary with summary information
        """
        with self._lock:
            return {
                "status": self._status,
                "drone": {
                    "connected": self._drone.connected,
                    "armed": self._drone.armed,
                    "mode": self._drone.mode,
                    "latitude": self._drone.latitude,
                    "longitude": self._drone.longitude,
                    "altitude": self._drone.altitude,
                    "altitude_relative": self._drone.altitude_relative,
                    "yaw": self._drone.yaw,
                    "roll": self._drone.roll,
                    "pitch": self._drone.pitch,
                    "vx": self._calculate_vx(),
                    "vy": self._calculate_vy(),
                    "vz": self._drone.climb_rate,
                    "battery_percent": self._drone.battery_remaining,
                    "gps_fix_type": self._drone.gps_fix,
                    "satellites_visible": self._drone.gps_satellites,
                },
                "gimbal": {
                    "connected": self._gimbal.connected,
                    "yaw": self._gimbal.yaw,
                    "pitch": self._gimbal.pitch,
                    "roll": self._gimbal.roll,
                    "locked": self._gimbal.locked_target_id is not None,
                },
                "camera": {
                    "connected": self._camera.connected,
                    "fps": self._camera.fps,
                    "resolution": self._camera.resolution,
                    "frame_count": self._camera.frame_count,
                },
                "detections": len(self._detections),
                "mission": {
                    "mode": self._mission.mode.value,
                    "active": self._mission.active,
                    "target": self._mission.target_id,
                },
                "errors": len(self._errors),
            }
