"""
MAVLink Interface for CubeOrange Flight Controller.

This module provides a comprehensive interface for communicating with the
CubeOrange autopilot via MAVLink protocol. It handles:
- Telemetry reception (position, attitude, battery, GPS)
- Command transmission (waypoints, mode changes, gimbal control)
- Mission management
- Heartbeat maintenance

The interface runs in a separate thread to handle asynchronous MAVLink messages.

Author: DroneBrain Team
"""

import time
import threading
import logging
from typing import Optional, Callable, Tuple, List
from pymavlink import mavutil

from ..core.state import SystemState

logger = logging.getLogger(__name__)


class MAVLinkInterface:
    """
    MAVLink communication interface for drone control.

    This class manages all MAVLink communication with the flight controller,
    including receiving telemetry and sending commands.

    Attributes:
        state: Reference to global SystemState
        connection: MAVLink connection object
        running: Thread running flag
        thread: Background telemetry thread

    Example:
        >>> state = SystemState()
        >>> mavlink = MAVLinkInterface(state, "192.168.144.10", 14550)
        >>> mavlink.connect()
        >>> mavlink.start()
        >>> # ... do work ...
        >>> mavlink.stop()
    """

    def __init__(
        self,
        state: SystemState,
        ip: str = "192.168.144.10",
        port: int = 14550,
        protocol: str = "udpout",
        system_id: int = 255,
        component_id: int = 190,
    ):
        """
        Initialize MAVLink interface.

        Args:
            state: Global system state manager
            ip: Flight controller IP address
            port: MAVLink port
            protocol: Connection protocol (udpin, udpout, tcp)
            system_id: GCS system ID
            component_id: GCS component ID
        """
        self.state = state
        self.ip = ip
        self.port = port
        self.protocol = protocol
        self.system_id = system_id
        self.component_id = component_id

        self.connection: Optional[mavutil.mavlink_connection] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None

        # Target system/component (will be identified from first received message)
        self.target_system = None
        self.target_component = None

        # Telemetry data cache
        self._last_heartbeat = 0
        self._heartbeat_interval = 1.0  # seconds

        # Callbacks for custom message handling
        self._message_callbacks = {}

    def connect(self) -> bool:
        """
        Establish MAVLink connection to flight controller.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            # Build connection string based on protocol
            if self.protocol == "udpin":
                # For UDPCI (UDP Client-Server):
                # Flight controller is UDP server listening on port
                # We are UDP client connecting to server
                # Use 'udpout' to connect to the server
                connection_string = f"udpout:{self.ip}:{self.port}"
                logger.info(f"Connecting to MAVLink (UDPCI client mode) at {connection_string}")
            else:
                # For other protocols, use as specified
                connection_string = f"{self.protocol}:{self.ip}:{self.port}"
                logger.info(f"Connecting to MAVLink at {connection_string}")

            self.connection = mavutil.mavlink_connection(
                connection_string,
                source_system=self.system_id,
                source_component=self.component_id,
            )

            logger.info("MAVLink connection established")
            logger.info("Will identify target system from received heartbeat...")

            # Connection established - target will be identified from first heartbeat
            # in the telemetry loop (don't block here)
            return True

        except Exception as e:
            logger.error(f"MAVLink connection error: {e}")
            self.state.add_error(f"MAVLink connection failed: {e}")
            return False

    def start(self):
        """Start background telemetry thread."""
        if self.running:
            logger.warning("MAVLink interface already running")
            return

        self.running = True
        self.thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.thread.start()

        # Send initial heartbeat to announce ourselves to UDP server
        # (UDP servers need to know client address before they can send back)
        logger.info("Sending initial heartbeat to announce ourselves...")
        self._send_heartbeat()
        time.sleep(0.1)  # Brief delay to let heartbeat be processed

        # Request data streams from flight controller
        self._request_data_streams()

        logger.info("MAVLink telemetry thread started")

    def stop(self):
        """Stop background telemetry thread."""
        if not self.running:
            return

        logger.info("Stopping MAVLink interface...")
        self.running = False

        if self.thread:
            self.thread.join(timeout=5.0)

        if self.connection:
            self.connection.close()

        self.state.update_drone_state(connected=False)
        logger.info("MAVLink interface stopped")

    def _telemetry_loop(self):
        """
        Main telemetry reception loop (runs in background thread).

        Continuously receives and processes MAVLink messages.
        """
        logger.info("Telemetry loop started")

        while self.running:
            try:
                # Check if connection exists
                if not self.connection:
                    logger.warning("No MAVLink connection, waiting...")
                    time.sleep(1.0)
                    continue

                # Send periodic heartbeat
                current_time = time.time()
                if current_time - self._last_heartbeat > self._heartbeat_interval:
                    self._send_heartbeat()
                    self._last_heartbeat = current_time

                # Receive messages with timeout
                msg = self.connection.recv_match(timeout=0.1)

                if msg:
                    self._handle_message(msg)

            except Exception as e:
                logger.error(f"Error in telemetry loop: {e}")
                time.sleep(0.1)

        logger.info("Telemetry loop stopped")

    def _send_heartbeat(self):
        """Send heartbeat to flight controller."""
        if not self.connection:
            return

        try:
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,  # Type: Ground Control Station
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,  # base_mode
                0,  # custom_mode
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
        except Exception as e:
            logger.error(f"Error sending heartbeat: {e}")

    def _request_data_streams(self):
        """Request telemetry streams from flight controller at specific rates."""
        if not self.connection:
            return

        try:
            # Request all data streams at 4 Hz (every 250ms)
            # Using legacy REQUEST_DATA_STREAM for compatibility with all autopilots
            streams = [
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
            ]

            for stream in streams:
                self.connection.mav.request_data_stream_send(
                    self.target_system if self.target_system else 1,  # Target system
                    self.target_component if self.target_component else 1,  # Target component
                    stream,  # Stream ID
                    4,  # Rate in Hz
                    1,  # Start streaming (1) or stop (0)
                )

            logger.info("Requested MAVLink data streams at 4 Hz")

        except Exception as e:
            logger.error(f"Error requesting data streams: {e}")

    def _handle_message(self, msg):
        """
        Process received MAVLink message.

        Args:
            msg: MAVLink message object
        """
        msg_type = msg.get_type()

        # Identify target system from first AUTOPILOT message (if not already identified)
        # Filter for autopilot component to avoid locking onto gimbal/camera
        if self.target_system is None:
            src_component = msg.get_srcComponent()
            # Only identify from autopilot (component ID 1)
            if src_component == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                self.target_system = msg.get_srcSystem()
                self.target_component = src_component
                logger.info(
                    f"Identified autopilot from {msg_type}: System {self.target_system}, "
                    f"Component {self.target_component}"
                )
                self.state.update_drone_state(connected=True)
            else:
                # Log but don't identify from non-autopilot components
                logger.debug(
                    f"Ignoring {msg_type} from non-autopilot component "
                    f"(System {msg.get_srcSystem()}, Component {src_component})"
                )
                return

        # Dispatch to appropriate handler
        handler_map = {
            "HEARTBEAT": self._handle_heartbeat,
            "GLOBAL_POSITION_INT": self._handle_global_position,
            "ATTITUDE": self._handle_attitude,
            "VFR_HUD": self._handle_vfr_hud,
            "SYS_STATUS": self._handle_sys_status,
            "GPS_RAW_INT": self._handle_gps_raw,
            "BATTERY_STATUS": self._handle_battery_status,
        }

        if msg_type in handler_map:
            handler_map[msg_type](msg)

        # Call custom callbacks if registered
        if msg_type in self._message_callbacks:
            self._message_callbacks[msg_type](msg)

    def _handle_heartbeat(self, msg):
        """Process HEARTBEAT message."""
        # Only process heartbeat from autopilot (component ID 1)
        # This prevents gimbal/camera heartbeats from changing flight mode
        if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
            return

        # Decode flight mode
        mode = mavutil.mode_string_v10(msg)

        # If mode_string_v10 returns opcode format, try to decode ArduPilot custom mode
        if mode.startswith("Mode("):
            # ArduPilot custom mode mapping for copter
            COPTER_MODES = {
                0: "STABILIZE",
                1: "ACRO",
                2: "ALT_HOLD",
                3: "AUTO",
                4: "GUIDED",
                5: "LOITER",
                6: "RTL",
                7: "CIRCLE",
                9: "LAND",
                11: "DRIFT",
                13: "SPORT",
                14: "FLIP",
                15: "AUTOTUNE",
                16: "POSHOLD",
                17: "BRAKE",
                18: "THROW",
                19: "AVOID_ADSB",
                20: "GUIDED_NOGPS",
                21: "SMART_RTL",
                22: "FLOWHOLD",
                23: "FOLLOW",
                24: "ZIGZAG",
                25: "SYSTEMID",
                26: "AUTOROTATE",
                27: "AUTO_RTL",
            }
            # Try to decode custom mode
            custom_mode = msg.custom_mode
            if custom_mode in COPTER_MODES:
                mode = COPTER_MODES[custom_mode]
            else:
                mode = f"CUSTOM_{custom_mode}"

        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

        self.state.update_drone_state(
            mode=mode,
            armed=bool(armed),
        )

    def _handle_global_position(self, msg):
        """Process GLOBAL_POSITION_INT message."""
        # MAVLink sends position in 1e7 degrees and mm
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0  # mm to meters
        alt_rel = msg.relative_alt / 1000.0

        heading = msg.hdg / 100.0  # centidegrees to degrees

        self.state.update_drone_position(
            lat=lat,
            lon=lon,
            alt=alt,
            alt_rel=alt_rel,
        )
        self.state.update_drone_state(heading=heading)

    def _handle_attitude(self, msg):
        """Process ATTITUDE message."""
        import math

        # Convert radians to degrees
        roll = math.degrees(msg.roll)
        pitch = math.degrees(msg.pitch)
        yaw = math.degrees(msg.yaw)

        self.state.update_drone_attitude(roll=roll, pitch=pitch, yaw=yaw)

    def _handle_vfr_hud(self, msg):
        """Process VFR_HUD message (speed and climb rate)."""
        self.state.update_drone_state(
            groundspeed=msg.groundspeed,
            airspeed=msg.airspeed,
            climb_rate=msg.climb,
            # Note: msg.alt is barometric altitude, not relative - use GLOBAL_POSITION_INT instead
            # heading is also better from GLOBAL_POSITION_INT (magnetic vs GPS heading)
        )

    def _handle_sys_status(self, msg):
        """Process SYS_STATUS message."""
        battery_voltage = msg.voltage_battery / 1000.0  # mV to V
        battery_current = msg.current_battery / 100.0  # cA to A
        battery_remaining = msg.battery_remaining

        self.state.update_drone_state(
            battery_voltage=battery_voltage,
            battery_current=battery_current,
            battery_remaining=battery_remaining,
        )

    def _handle_gps_raw(self, msg):
        """Process GPS_RAW_INT message."""
        self.state.update_drone_state(
            gps_fix=msg.fix_type,
            gps_satellites=msg.satellites_visible,
        )

    def _handle_battery_status(self, msg):
        """Process BATTERY_STATUS message (more detailed than SYS_STATUS)."""
        if len(msg.voltages) > 0:
            # Sum all cell voltages (mV to V)
            total_voltage = sum(v for v in msg.voltages if v != 65535) / 1000.0
            self.state.update_drone_state(battery_voltage=total_voltage)

    # ===== Command Methods =====

    def arm(self) -> bool:
        """
        Arm the vehicle.

        Returns:
            True if command sent successfully
        """
        return self._arm_disarm(True)

    def disarm(self) -> bool:
        """
        Disarm the vehicle.

        Returns:
            True if command sent successfully
        """
        return self._arm_disarm(False)

    def _arm_disarm(self, arm: bool) -> bool:
        """
        Send arm/disarm command.

        Args:
            arm: True to arm, False to disarm

        Returns:
            True if successful
        """
        if not self.connection:
            logger.error("No MAVLink connection")
            return False

        try:
            self.connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                1 if arm else 0,  # param1: 1=arm, 0=disarm
                0,
                0,
                0,
                0,
                0,
                0,  # unused params
            )

            logger.info(f"Sent {'ARM' if arm else 'DISARM'} command")
            return True

        except Exception as e:
            logger.error(f"Error sending arm/disarm command: {e}")
            return False

    def set_mode(self, mode: str) -> bool:
        """
        Set flight mode.

        Args:
            mode: Mode name (e.g., "GUIDED", "AUTO", "LOITER", "RTL")

        Returns:
            True if successful
        """
        if not self.connection:
            logger.error("No MAVLink connection")
            return False

        try:
            # Get mode ID
            mode_id = self.connection.mode_mapping().get(mode.upper())

            if mode_id is None:
                logger.error(f"Unknown mode: {mode}")
                return False

            self.connection.mav.set_mode_send(
                self.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )

            logger.info(f"Set mode to {mode}")
            return True

        except Exception as e:
            logger.error(f"Error setting mode: {e}")
            return False

    def goto_position(
        self,
        lat: float,
        lon: float,
        alt: float,
        yaw: Optional[float] = None,
    ) -> bool:
        """
        Send vehicle to GPS position.

        Args:
            lat: Latitude (degrees)
            lon: Longitude (degrees)
            alt: Altitude (meters, relative to home)
            yaw: Optional heading (degrees)

        Returns:
            True if successful
        """
        if not self.connection:
            logger.error("No MAVLink connection")
            return False

        try:
            # Convert to MAVLink format
            lat_int = int(lat * 1e7)
            lon_int = int(lon * 1e7)

            # Type mask (ignore velocity and acceleration)
            type_mask = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            )

            if yaw is None:
                type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE

            self.connection.mav.set_position_target_global_int_send(
                0,  # time_boot_ms (not used)
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                lat_int,
                lon_int,
                alt,
                0,
                0,
                0,  # vx, vy, vz
                0,
                0,
                0,  # afx, afy, afz
                yaw if yaw is not None else 0,
                0,  # yaw_rate
            )

            logger.info(f"Sent GOTO command: {lat:.6f}, {lon:.6f}, {alt:.1f}m")
            return True

        except Exception as e:
            logger.error(f"Error sending goto command: {e}")
            return False

    def upload_mission(self, waypoints: List[Tuple[float, float, float]]) -> bool:
        """
        Upload a mission (list of waypoints).

        Args:
            waypoints: List of (lat, lon, alt) tuples

        Returns:
            True if successful
        """
        if not self.connection:
            logger.error("No MAVLink connection")
            return False

        try:
            # Clear existing mission
            self.connection.waypoint_clear_all_send()
            time.sleep(0.5)

            # Send mission count
            self.connection.waypoint_count_send(len(waypoints))

            # Upload each waypoint
            for i, (lat, lon, alt) in enumerate(waypoints):
                # Wait for request
                msg = self.connection.recv_match(
                    type="MISSION_REQUEST",
                    blocking=True,
                    timeout=5,
                )

                if not msg or msg.seq != i:
                    logger.error(f"Mission upload failed at waypoint {i}")
                    return False

                # Send waypoint
                self.connection.mav.mission_item_int_send(
                    self.target_system,
                    self.target_component,
                    i,  # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0 if i > 0 else 1,  # current (1 for first waypoint)
                    1,  # autocontinue
                    0,
                    0,
                    0,
                    0,  # params 1-4 (unused for waypoint)
                    int(lat * 1e7),
                    int(lon * 1e7),
                    alt,
                )

            # Wait for acknowledgment
            msg = self.connection.recv_match(
                type="MISSION_ACK",
                blocking=True,
                timeout=5,
            )

            if msg and msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                logger.info(f"Mission uploaded: {len(waypoints)} waypoints")
                return True
            else:
                logger.error("Mission upload failed - no ACK")
                return False

        except Exception as e:
            logger.error(f"Error uploading mission: {e}")
            return False

    def set_gimbal_angle(self, pitch: float, yaw: float = 0) -> bool:
        """
        Control gimbal via MAVLink (if supported).

        Note: The Siyi gimbal is controlled via its own UDP protocol,
        but this method is provided for autopilot-connected gimbals.

        Args:
            pitch: Pitch angle (degrees, -90 to 0)
            yaw: Yaw angle (degrees)

        Returns:
            True if successful
        """
        if not self.connection:
            return False

        try:
            self.connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                0,
                pitch,  # pitch
                0,  # roll
                yaw,  # yaw
                0,
                0,
                0,
                mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
            )

            logger.debug(f"Sent gimbal command: pitch={pitch}, yaw={yaw}")
            return True

        except Exception as e:
            logger.error(f"Error sending gimbal command: {e}")
            return False

    # ===== Utility Methods =====

    def register_message_callback(self, msg_type: str, callback: Callable):
        """
        Register a callback for custom message handling.

        Args:
            msg_type: MAVLink message type (e.g., "STATUSTEXT")
            callback: Function to call with message
        """
        self._message_callbacks[msg_type] = callback

    def get_home_position(self) -> Optional[Tuple[float, float, float]]:
        """
        Get home position from autopilot.

        Returns:
            Tuple of (lat, lon, alt) or None if not available
        """
        if not self.connection:
            return None

        try:
            # Request home position
            self.connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )

            # Wait for response
            msg = self.connection.recv_match(
                type="HOME_POSITION",
                blocking=True,
                timeout=2,
            )

            if msg:
                lat = msg.latitude / 1e7
                lon = msg.longitude / 1e7
                alt = msg.altitude / 1000.0
                return (lat, lon, alt)

        except Exception as e:
            logger.error(f"Error getting home position: {e}")

        return None

    def is_armed(self) -> bool:
        """
        Check if vehicle is armed.

        Returns:
            True if armed
        """
        return self.state.get_drone_state().armed

    def get_mode(self) -> str:
        """
        Get current flight mode.

        Returns:
            Mode name string
        """
        return self.state.get_drone_state().mode
