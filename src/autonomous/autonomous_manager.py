"""
Autonomous Flight Behavior Manager.

Orchestrates autonomous flight modes (Loiter, Follow) based on locked targets.
Generates waypoints and sends them to the flight controller via MAVLink.

Author: DroneBrain Team
"""

import time
import math
import logging
from typing import Optional, Tuple, Dict, Any
from enum import Enum
from dataclasses import dataclass

from ..core.state import SystemState
from ..interfaces.mavlink_interface import MAVLinkInterface


logger = logging.getLogger(__name__)


class AutonomousMode(Enum):
    """Autonomous flight modes."""
    DISABLED = "disabled"
    LOITER = "loiter"  # Orbit around target at standoff distance
    FOLLOW = "follow"  # Follow moving target maintaining standoff distance


@dataclass
class TargetInfo:
    """Information about the tracked target."""
    track_id: int
    latitude: float
    longitude: float
    last_seen: float  # timestamp
    distance: float  # meters from drone


class AutonomousManager:
    """
    Manages autonomous flight behaviors.

    This class monitors locked targets and generates waypoints for:
    - Loiter mode: Circular orbit around stationary target
    - Follow mode: Dynamic following of moving target

    Attributes:
        state: Reference to global SystemState
        mavlink: MAVLink interface for flight control
        config: Autonomous configuration parameters
        mode: Current autonomous mode
        running: Manager running flag
    """

    def __init__(
        self,
        state: SystemState,
        mavlink: MAVLinkInterface,
        config: Dict[str, Any],
    ):
        """
        Initialize autonomous manager.

        Args:
            state: Global system state manager
            mavlink: MAVLink interface for flight control
            config: Configuration dict with autonomous parameters
        """
        self.state = state
        self.mavlink = mavlink
        self.config = config

        # Current mode
        self.mode = AutonomousMode.DISABLED

        # Target tracking
        self.target_info: Optional[TargetInfo] = None
        self.last_target_position: Optional[Tuple[float, float]] = None

        # Waypoint generation
        self.last_waypoint_time = 0
        self.waypoint_update_interval = config.get('waypoint_update_interval', 2.0)
        self.min_target_movement = config.get('min_target_movement', 5.0)

        # Configuration parameters
        self.standoff_distance = config.get('standoff_distance', 50.0)
        self.loiter_altitude_offset = config.get('loiter_altitude_offset', 0.0)
        self.loiter_speed = config.get('loiter_speed', 5.0)
        self.follow_altitude_mode = config.get('follow_altitude_mode', 'maintain')
        self.follow_speed = config.get('follow_speed', 8.0)

        # Loiter state
        self.loiter_start_bearing: Optional[float] = None
        self.loiter_altitude: Optional[float] = None

        self.running = False

        logger.info(
            f"Autonomous Manager initialized: "
            f"standoff={self.standoff_distance}m, "
            f"loiter_speed={self.loiter_speed}m/s, "
            f"follow_speed={self.follow_speed}m/s"
        )

    def set_mode(self, mode: AutonomousMode) -> bool:
        """
        Set autonomous flight mode.

        Args:
            mode: Desired autonomous mode

        Returns:
            True if mode set successfully
        """
        if mode == self.mode:
            return True

        # Validate state before enabling autonomous modes
        if mode != AutonomousMode.DISABLED:
            # Check if drone is armed and in GUIDED mode
            drone_state = self.state.get_drone_state()
            if not drone_state.armed:
                logger.error("Cannot enable autonomous mode: Drone not armed")
                return False

            if drone_state.mode != "GUIDED":
                logger.warning(f"Drone mode is {drone_state.mode}, switching to GUIDED")
                if not self.mavlink.set_mode("GUIDED"):
                    logger.error("Failed to set GUIDED mode")
                    return False

            # Check if we have a locked target
            tracking_state = self.state.get_tracking_state()
            if not tracking_state.is_tracking:
                logger.error("Cannot enable autonomous mode: No target locked")
                return False

        old_mode = self.mode
        self.mode = mode

        logger.info(f"Autonomous mode changed: {old_mode.value} → {mode.value}")

        # Mode-specific initialization
        if mode == AutonomousMode.LOITER:
            self._init_loiter_mode()
        elif mode == AutonomousMode.FOLLOW:
            self._init_follow_mode()
        elif mode == AutonomousMode.DISABLED:
            self._cleanup_autonomous()

        return True

    def update(self, detections: list) -> None:
        """
        Update autonomous behavior based on current detections.

        Called from the main vision processing loop.

        Args:
            detections: List of Detection objects from vision pipeline
        """
        if self.mode == AutonomousMode.DISABLED:
            return

        # Update target information from detections
        self._update_target_info(detections)

        # Check if we should generate new waypoints
        current_time = time.time()
        if current_time - self.last_waypoint_time < self.waypoint_update_interval:
            return

        # Generate waypoints based on mode
        if self.mode == AutonomousMode.LOITER:
            self._update_loiter()
        elif self.mode == AutonomousMode.FOLLOW:
            self._update_follow()

        self.last_waypoint_time = current_time

    def _update_target_info(self, detections: list) -> None:
        """
        Update tracked target information from detections.

        Args:
            detections: List of Detection objects
        """
        tracking_state = self.state.get_tracking_state()

        if not tracking_state.is_tracking:
            self.target_info = None
            return

        # Find the locked target in detections
        locked_id = tracking_state.locked_track_id

        for det in detections:
            if det.track_id == locked_id and det.latitude is not None:
                self.target_info = TargetInfo(
                    track_id=det.track_id,
                    latitude=det.latitude,
                    longitude=det.longitude,
                    last_seen=time.time(),
                    distance=det.distance if hasattr(det, 'distance') else 0.0,
                )
                return

        # Target not found in current detections
        # Keep last known position for a grace period
        if self.target_info and (time.time() - self.target_info.last_seen) > 5.0:
            logger.warning("Target lost for >5 seconds, disabling autonomous mode")
            self.set_mode(AutonomousMode.DISABLED)

    def _init_loiter_mode(self) -> None:
        """Initialize loiter mode parameters."""
        drone_state = self.state.get_drone_state()

        # Store current altitude for loiter
        self.loiter_altitude = drone_state.alt_rel + self.loiter_altitude_offset

        # Calculate initial bearing to target
        if self.target_info:
            drone_lat, drone_lon = drone_state.lat, drone_state.lon
            target_lat, target_lon = self.target_info.latitude, self.target_info.longitude
            self.loiter_start_bearing = self._calculate_bearing(
                drone_lat, drone_lon, target_lat, target_lon
            )
        else:
            self.loiter_start_bearing = 0.0

        logger.info(
            f"Loiter mode initialized: "
            f"altitude={self.loiter_altitude:.1f}m, "
            f"start_bearing={self.loiter_start_bearing:.1f}°"
        )

    def _init_follow_mode(self) -> None:
        """Initialize follow mode parameters."""
        if self.target_info:
            self.last_target_position = (
                self.target_info.latitude,
                self.target_info.longitude,
            )
        logger.info("Follow mode initialized")

    def _cleanup_autonomous(self) -> None:
        """Clean up autonomous mode state."""
        self.target_info = None
        self.last_target_position = None
        self.loiter_start_bearing = None
        self.loiter_altitude = None
        logger.info("Autonomous mode disabled")

    def _update_loiter(self) -> None:
        """Update loiter orbit behavior."""
        if not self.target_info:
            return

        # Generate circular orbit waypoint
        # Calculate next position on orbit circle
        drone_state = self.state.get_drone_state()
        current_bearing = self._calculate_bearing(
            drone_state.lat, drone_state.lon,
            self.target_info.latitude, self.target_info.longitude
        )

        # Move to next orbit position (advance by angle based on speed and radius)
        # Angular velocity = linear_velocity / radius
        orbit_period = (2 * math.pi * self.standoff_distance) / self.loiter_speed
        angle_increment = (360.0 / orbit_period) * self.waypoint_update_interval

        next_bearing = (current_bearing + angle_increment) % 360.0

        # Calculate next waypoint position on orbit circle
        next_lat, next_lon = self._calculate_destination(
            self.target_info.latitude,
            self.target_info.longitude,
            next_bearing,
            self.standoff_distance,
        )

        # Send goto command
        success = self.mavlink.goto_position(
            next_lat,
            next_lon,
            self.loiter_altitude,
            yaw=None,  # Let drone auto-orient
        )

        if success:
            logger.debug(
                f"Loiter waypoint: {next_lat:.6f}, {next_lon:.6f}, "
                f"alt={self.loiter_altitude:.1f}m, bearing={next_bearing:.1f}°"
            )

    def _update_follow(self) -> None:
        """Update follow behavior."""
        if not self.target_info:
            return

        # Check if target has moved significantly
        if self.last_target_position:
            distance_moved = self._calculate_distance(
                self.last_target_position[0],
                self.last_target_position[1],
                self.target_info.latitude,
                self.target_info.longitude,
            )

            if distance_moved < self.min_target_movement:
                logger.debug(f"Target moved only {distance_moved:.1f}m, skipping waypoint update")
                return

        # Calculate follow position (standoff distance behind target)
        drone_state = self.state.get_drone_state()

        # Bearing from drone to target
        bearing_to_target = self._calculate_bearing(
            drone_state.lat, drone_state.lon,
            self.target_info.latitude, self.target_info.longitude
        )

        # Calculate follow position: standoff distance from target
        # Position ourselves at standoff distance from target
        follow_lat, follow_lon = self._calculate_destination(
            self.target_info.latitude,
            self.target_info.longitude,
            (bearing_to_target + 180) % 360,  # Opposite side
            self.standoff_distance,
        )

        # Altitude handling
        if self.follow_altitude_mode == 'maintain':
            follow_alt = drone_state.alt_rel
        else:
            # Could implement altitude matching based on target size/distance
            follow_alt = drone_state.alt_rel

        # Send goto command
        success = self.mavlink.goto_position(
            follow_lat,
            follow_lon,
            follow_alt,
            yaw=None,
        )

        if success:
            logger.info(
                f"Follow waypoint: {follow_lat:.6f}, {follow_lon:.6f}, "
                f"alt={follow_alt:.1f}m (target moved {distance_moved:.1f}m)"
                if self.last_target_position else
                f"Follow waypoint: {follow_lat:.6f}, {follow_lon:.6f}, alt={follow_alt:.1f}m"
            )

        # Update last position
        self.last_target_position = (
            self.target_info.latitude,
            self.target_info.longitude,
        )

    # ===== Utility Methods =====

    @staticmethod
    def _calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate bearing between two GPS coordinates.

        Args:
            lat1, lon1: Start position (degrees)
            lat2, lon2: End position (degrees)

        Returns:
            Bearing in degrees (0-360)
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlon_rad = math.radians(lon2 - lon1)

        y = math.sin(dlon_rad) * math.cos(lat2_rad)
        x = (
            math.cos(lat1_rad) * math.sin(lat2_rad)
            - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad)
        )

        bearing_rad = math.atan2(y, x)
        bearing_deg = math.degrees(bearing_rad)

        return (bearing_deg + 360) % 360

    @staticmethod
    def _calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate distance between two GPS coordinates using Haversine formula.

        Args:
            lat1, lon1: Start position (degrees)
            lat2, lon2: End position (degrees)

        Returns:
            Distance in meters
        """
        R = 6371000  # Earth radius in meters

        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat_rad = math.radians(lat2 - lat1)
        dlon_rad = math.radians(lon2 - lon1)

        a = (
            math.sin(dlat_rad / 2) ** 2
            + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon_rad / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    @staticmethod
    def _calculate_destination(
        lat: float, lon: float, bearing: float, distance: float
    ) -> Tuple[float, float]:
        """
        Calculate destination point given start point, bearing, and distance.

        Args:
            lat, lon: Start position (degrees)
            bearing: Bearing in degrees (0-360)
            distance: Distance in meters

        Returns:
            Tuple of (lat, lon) in degrees
        """
        R = 6371000  # Earth radius in meters

        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        bearing_rad = math.radians(bearing)

        lat2_rad = math.asin(
            math.sin(lat_rad) * math.cos(distance / R)
            + math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad)
        )

        lon2_rad = lon_rad + math.atan2(
            math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
            math.cos(distance / R) - math.sin(lat_rad) * math.sin(lat2_rad),
        )

        lat2 = math.degrees(lat2_rad)
        lon2 = math.degrees(lon2_rad)

        return (lat2, lon2)

    def get_status(self) -> Dict[str, Any]:
        """
        Get current autonomous status.

        Returns:
            Status dictionary
        """
        return {
            'mode': self.mode.value,
            'enabled': self.mode != AutonomousMode.DISABLED,
            'standoff_distance': self.standoff_distance,
            'target_locked': self.target_info is not None,
            'target_info': {
                'track_id': self.target_info.track_id,
                'latitude': self.target_info.latitude,
                'longitude': self.target_info.longitude,
                'distance': self.target_info.distance,
                'last_seen': self.target_info.last_seen,
            } if self.target_info else None,
        }
