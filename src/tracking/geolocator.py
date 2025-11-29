"""
Geolocator Module - Target GPS Coordinate Calculation

Calculates real-world GPS coordinates for detected objects using photogrammetry.
Accounts for drone position, attitude, gimbal angles, and camera parameters.

The math:
1. Convert pixel coordinates to normalized image coordinates
2. Apply camera intrinsics to get bearing angles
3. Transform through gimbal angles
4. Transform through drone attitude (roll, pitch, yaw)
5. Project ray to ground plane (assuming targets at ground level)
6. Calculate GPS offset from drone position
"""

import logging
import math
from typing import Optional, Tuple
from dataclasses import dataclass

logger = logging.getLogger(__name__)

# WGS84 Earth constants
EARTH_RADIUS = 6378137.0  # meters (equatorial radius)


@dataclass
class CameraIntrinsics:
    """Camera intrinsic parameters for photogrammetry calculations."""

    focal_length_px: float  # Focal length in pixels
    sensor_width_px: int  # Sensor width in pixels
    sensor_height_px: int  # Sensor height in pixels
    cx: float  # Principal point x (usually width/2)
    cy: float  # Principal point y (usually height/2)

    @property
    def horizontal_fov(self) -> float:
        """Calculate horizontal field of view in radians."""
        return 2 * math.atan(self.sensor_width_px / (2 * self.focal_length_px))

    @property
    def vertical_fov(self) -> float:
        """Calculate vertical field of view in radians."""
        return 2 * math.atan(self.sensor_height_px / (2 * self.focal_length_px))


class Geolocator:
    """
    Calculates GPS coordinates for detected objects using photogrammetry.

    Transforms pixel coordinates through camera, gimbal, and drone coordinate
    systems to calculate real-world GPS positions.

    Example:
        >>> intrinsics = CameraIntrinsics(
        ...     focal_length_px=1280,
        ...     sensor_width_px=1280,
        ...     sensor_height_px=720,
        ...     cx=640, cy=360
        ... )
        >>> geolocator = Geolocator(intrinsics)
        >>> lat, lon = geolocator.pixel_to_gps(
        ...     pixel_x=640, pixel_y=360,
        ...     drone_lat=36.7765, drone_lon=-75.9622, drone_alt=100,
        ...     drone_roll=0, drone_pitch=0, drone_yaw=90,
        ...     gimbal_pitch=-45, gimbal_yaw=0
        ... )
    """

    def __init__(self, camera_intrinsics: CameraIntrinsics):
        """
        Initialize geolocator with camera parameters.

        Args:
            camera_intrinsics: Camera intrinsic parameters
        """
        self.intrinsics = camera_intrinsics
        logger.info(
            f"Geolocator initialized: "
            f"HFOV={math.degrees(self.intrinsics.horizontal_fov):.1f}°, "
            f"VFOV={math.degrees(self.intrinsics.vertical_fov):.1f}°"
        )

    def pixel_to_gps(
        self,
        pixel_x: float,
        pixel_y: float,
        drone_lat: float,
        drone_lon: float,
        drone_alt: float,
        drone_roll: float,
        drone_pitch: float,
        drone_yaw: float,
        gimbal_pitch: float,
        gimbal_yaw: float,
        ground_elevation: float = 0.0,
    ) -> Optional[Tuple[float, float]]:
        """
        Convert pixel coordinates to GPS coordinates.

        Args:
            pixel_x: Target x coordinate in image (pixels)
            pixel_y: Target y coordinate in image (pixels)
            drone_lat: Drone latitude (degrees)
            drone_lon: Drone longitude (degrees)
            drone_alt: Drone altitude above ground (meters)
            drone_roll: Drone roll angle (degrees, + right wing down)
            drone_pitch: Drone pitch angle (degrees, + nose up)
            drone_yaw: Drone yaw/heading angle (degrees, 0=North, clockwise)
            gimbal_pitch: Gimbal pitch angle (degrees, + up, - down)
            gimbal_yaw: Gimbal yaw angle (degrees, + right)
            ground_elevation: Ground elevation at target (meters, default 0)

        Returns:
            Tuple of (latitude, longitude) in degrees, or None if calculation fails
        """
        # Step 1: Convert pixel to normalized image coordinates
        # Origin at principal point, y-axis pointing down
        x_norm = (pixel_x - self.intrinsics.cx) / self.intrinsics.focal_length_px
        y_norm = (pixel_y - self.intrinsics.cy) / self.intrinsics.focal_length_px

        # Step 2: Calculate bearing angles in camera frame
        # Camera frame: x=right, y=down, z=forward
        bearing_x = math.atan(x_norm)  # Horizontal angle (radians)
        bearing_y = math.atan(y_norm)  # Vertical angle (radians)

        # Step 3: Create unit vector in camera frame pointing to target
        # This is the direction from camera to target
        cam_x = math.sin(bearing_x) * math.cos(bearing_y)
        cam_y = math.sin(bearing_y)
        cam_z = math.cos(bearing_x) * math.cos(bearing_y)

        # Step 4: Transform through gimbal angles
        # Gimbal pitch rotation (around x-axis)
        gimbal_pitch_rad = math.radians(gimbal_pitch)
        gim_x = cam_x
        gim_y = cam_y * math.cos(gimbal_pitch_rad) - cam_z * math.sin(
            gimbal_pitch_rad
        )
        gim_z = cam_y * math.sin(gimbal_pitch_rad) + cam_z * math.cos(
            gimbal_pitch_rad
        )

        # Gimbal yaw rotation (around z-axis)
        gimbal_yaw_rad = math.radians(gimbal_yaw)
        body_x = gim_x * math.cos(gimbal_yaw_rad) - gim_y * math.sin(gimbal_yaw_rad)
        body_y = gim_x * math.sin(gimbal_yaw_rad) + gim_y * math.cos(gimbal_yaw_rad)
        body_z = gim_z

        # Step 5: Transform through drone attitude (body frame to NED frame)
        # NED frame: x=North, y=East, z=Down
        # Apply yaw (heading) rotation
        yaw_rad = math.radians(drone_yaw)
        ned_x_temp = body_x * math.cos(yaw_rad) + body_y * math.sin(yaw_rad)
        ned_y_temp = -body_x * math.sin(yaw_rad) + body_y * math.cos(yaw_rad)
        ned_z_temp = body_z

        # Apply pitch rotation
        pitch_rad = math.radians(drone_pitch)
        ned_x = ned_x_temp * math.cos(pitch_rad) - ned_z_temp * math.sin(pitch_rad)
        ned_z_temp2 = ned_x_temp * math.sin(pitch_rad) + ned_z_temp * math.cos(
            pitch_rad
        )

        # Apply roll rotation
        roll_rad = math.radians(drone_roll)
        ned_y = ned_y_temp * math.cos(roll_rad) + ned_z_temp2 * math.sin(roll_rad)
        ned_z = -ned_y_temp * math.sin(roll_rad) + ned_z_temp2 * math.cos(roll_rad)

        # Step 6: Project ray to ground plane
        # We have a unit vector (ned_x, ned_y, ned_z) pointing from drone to target
        # Drone is at altitude drone_alt, target is at ground_elevation
        height_diff = drone_alt - ground_elevation

        # Check if looking upward (can't intersect ground)
        if ned_z <= 0:
            logger.debug(
                f"Ray pointing upward (ned_z={ned_z:.3f}), cannot geolocate"
            )
            return None

        # Calculate scale factor to reach ground
        # ned_z is positive (down), so this will be positive
        scale = height_diff / ned_z

        # Calculate horizontal distances
        north_offset = ned_x * scale  # meters north
        east_offset = ned_y * scale  # meters east

        # Step 7: Convert offsets to GPS coordinates
        # Simple flat-earth approximation (good enough for local distances < 10km)
        lat_offset = north_offset / EARTH_RADIUS * (180 / math.pi)
        lon_offset = east_offset / (
            EARTH_RADIUS * math.cos(math.radians(drone_lat))
        ) * (180 / math.pi)

        target_lat = drone_lat + lat_offset
        target_lon = drone_lon + lon_offset

        return target_lat, target_lon

    def calculate_slant_range(
        self,
        drone_alt: float,
        drone_pitch: float,
        gimbal_pitch: float,
        bearing_y: float,
        ground_elevation: float = 0.0,
    ) -> Optional[float]:
        """
        Calculate slant range (direct distance) to target.

        Args:
            drone_alt: Drone altitude above ground (meters)
            drone_pitch: Drone pitch angle (degrees)
            gimbal_pitch: Gimbal pitch angle (degrees)
            bearing_y: Vertical bearing angle from camera center (radians)
            ground_elevation: Ground elevation (meters)

        Returns:
            Slant range in meters, or None if looking upward
        """
        # Total pitch angle (positive = looking up)
        total_pitch = drone_pitch + gimbal_pitch + math.degrees(bearing_y)
        total_pitch_rad = math.radians(total_pitch)

        # Can't calculate range if looking up
        if total_pitch_rad >= 0:
            return None

        # Height difference
        height_diff = drone_alt - ground_elevation

        # Slant range = height / sin(depression_angle)
        slant_range = height_diff / math.sin(abs(total_pitch_rad))

        return slant_range
