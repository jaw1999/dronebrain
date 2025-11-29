"""
Siyi A8 Mini Gimbal Interface.

This module provides a comprehensive interface for controlling the Siyi A8 Mini
gimbal camera over Ethernet using the custom SIYI UDP protocol.

Features:
- Gimbal attitude control (pitch/yaw)
- Telemetry reception (current angles)
- Photo and video recording commands
- Center/reset commands
- Thread-safe operation with state updates

Protocol Details:
- UDP communication on port 37260
- Packet structure: STX(0x6655) + CTRL + LEN + SEQ + CMD + DATA + CRC16
- Little-endian byte order
- CRC16 checksum validation

Author: DroneBrain Team
"""

import socket
import struct
import threading
import logging
import time
from typing import Optional, Dict, Any

from ..core.state import SystemState

logger = logging.getLogger(__name__)


class SiyiInterface:
    """
    Interface for Siyi A8 Mini gimbal camera control.

    This class handles all communication with the Siyi gimbal including
    attitude control, telemetry updates, and camera functions.

    Attributes:
        state: Reference to global SystemState
        ip: Gimbal IP address
        port: UDP control port
        sock: UDP socket
        running: Thread running flag
        thread: Background telemetry thread

    Example:
        >>> state = SystemState()
        >>> siyi = SiyiInterface(state, "192.168.144.25", 37260)
        >>> siyi.connect()
        >>> siyi.start()
        >>> siyi.set_attitude(yaw=30.0, pitch=-10.0)
        >>> siyi.stop()
    """

    # Protocol constants
    STX = 0x6655  # Start bytes (little-endian: 0x55, 0x66)

    # Command IDs
    CMD_ACQUIRE_FW_VER = 0x01
    CMD_ACQUIRE_HW_ID = 0x02
    CMD_AUTO_FOCUS = 0x04
    CMD_MANUAL_ZOOM = 0x05
    CMD_MANUAL_FOCUS = 0x06
    CMD_GIMBAL_ROTATION = 0x07
    CMD_CENTER_GIMBAL = 0x08
    CMD_ACQUIRE_GIMBAL_INFO = 0x0A
    CMD_FUNCTION_FEEDBACK = 0x0B
    CMD_PHOTO = 0x0C
    CMD_RECORD = 0x0D
    CMD_SET_GIMBAL_ATTITUDE = 0x0E

    def __init__(
        self,
        state: SystemState,
        ip: str = "192.168.144.25",
        port: int = 37260,
    ):
        """
        Initialize Siyi gimbal interface.

        Args:
            state: Global system state manager
            ip: Gimbal IP address
            port: UDP control port
        """
        self.state = state
        self.ip = ip
        self.port = port

        self.sock: Optional[socket.socket] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None

        # Timeout warning throttling
        self._timeout_count = 0
        self._timeout_warn_interval = 10  # Warn every 10th timeout

        # Sequence counter for packets
        self.seq = 0
        self._seq_lock = threading.Lock()

        # Telemetry polling interval
        self._telemetry_interval = 0.2  # 5 Hz

        # Rate limiting for attitude commands
        self._last_attitude_command = 0.0  # Timestamp of last command
        self._min_attitude_interval = 0.05  # Minimum 50ms between commands (20Hz max)

    def connect(self) -> bool:
        """
        Establish connection to gimbal.

        Returns:
            True if connection successful
        """
        try:
            logger.info(f"Connecting to Siyi gimbal at {self.ip}:{self.port}")

            # Create UDP socket
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(2.0)  # Use longer timeout for initial connection test

            # Test connection by getting gimbal info
            response = self._send_command(self.CMD_ACQUIRE_GIMBAL_INFO)

            if response:
                logger.info("Successfully connected to Siyi gimbal")
                self.state.update_gimbal_state(connected=True)
                # Now reduce timeout for normal operations
                self.sock.settimeout(0.2)
                return True
            else:
                logger.error("Failed to communicate with gimbal")
                return False

        except Exception as e:
            logger.error(f"Gimbal connection error: {e}")
            self.state.add_error(f"Siyi connection failed: {e}")
            return False

    def start(self):
        """Start background telemetry thread."""
        if self.running:
            logger.warning("Siyi interface already running")
            return

        self.running = True
        self.thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.thread.start()
        logger.info("Siyi telemetry thread started")

    def stop(self):
        """Stop background telemetry thread and close connection."""
        if not self.running:
            return

        logger.info("Stopping Siyi interface...")
        self.running = False

        if self.thread:
            self.thread.join(timeout=5.0)

        if self.sock:
            self.sock.close()

        self.state.update_gimbal_state(connected=False)
        logger.info("Siyi interface stopped")

    def _telemetry_loop(self):
        """
        Background thread to poll gimbal telemetry.

        Continuously requests gimbal attitude information and updates state.
        """
        logger.info("Siyi telemetry loop started")

        while self.running:
            try:
                # Request gimbal info
                response = self._send_command(self.CMD_ACQUIRE_GIMBAL_INFO)

                if response and len(response["data"]) >= 6:
                    # Parse yaw, pitch, roll (int16 in 0.1 degree units)
                    yaw, pitch, roll = struct.unpack("<hhh", response["data"][:6])

                    # Update state
                    self.state.update_gimbal_state(
                        yaw=yaw / 10.0,
                        pitch=pitch / 10.0,
                        roll=roll / 10.0,
                    )

                # Wait before next poll
                time.sleep(self._telemetry_interval)

            except Exception as e:
                logger.error(f"Error in Siyi telemetry loop: {e}")
                time.sleep(1.0)

        logger.info("Siyi telemetry loop stopped")

    # ===== Protocol Methods =====

    def _calc_crc16(self, data: bytes) -> int:
        """
        Calculate CRC16 checksum (CRC-16/XMODEM).

        Args:
            data: Bytes to calculate CRC for

        Returns:
            16-bit CRC value
        """
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
            crc &= 0xFFFF
        return crc

    def _build_packet(self, cmd_id: int, data: bytes = b"", need_ack: bool = True) -> bytes:
        """
        Build a SIYI protocol packet.

        Args:
            cmd_id: Command ID byte
            data: Data payload (bytes)
            need_ack: Whether to request acknowledgment

        Returns:
            Complete packet as bytes
        """
        # CTRL byte: bit 0 = need_ack
        ctrl = 0x01 if need_ack else 0x00

        # Data length (little-endian)
        data_len = len(data)

        # Sequence number (little-endian, 0-65535)
        with self._seq_lock:
            seq = self.seq % 65536
            self.seq += 1

        # Build packet without CRC
        packet = (
            struct.pack(
                "<HBHHB",
                self.STX,  # Start bytes (0x6655)
                ctrl,  # Control byte
                data_len,  # Data length
                seq,  # Sequence number
                cmd_id,  # Command ID
            )
            + data
        )

        # Calculate and append CRC16
        crc = self._calc_crc16(packet)
        packet += struct.pack("<H", crc)

        return packet

    def _parse_response(self, packet: bytes) -> Optional[Dict[str, Any]]:
        """
        Parse response packet.

        Args:
            packet: Raw packet bytes

        Returns:
            Dict with parsed response data or None if invalid
        """
        if len(packet) < 10:
            return None

        # Parse header
        stx, ctrl, _data_len, seq, cmd_id = struct.unpack("<HBHHB", packet[:8])

        # Verify start bytes
        if stx != self.STX:
            logger.warning(f"Invalid start bytes: {stx:04x}")
            return None

        # Extract data
        data = packet[8:-2]

        # Verify CRC
        expected_crc = struct.unpack("<H", packet[-2:])[0]
        actual_crc = self._calc_crc16(packet[:-2])

        if expected_crc != actual_crc:
            logger.warning(f"CRC mismatch: expected {expected_crc:04x}, got {actual_crc:04x}")
            return None

        return {"cmd_id": cmd_id, "seq": seq, "data": data, "is_ack": bool(ctrl & 0x02)}

    def _send_command(
        self, cmd_id: int, data: bytes = b"", need_ack: bool = True
    ) -> Optional[Dict[str, Any]]:
        """
        Send command to gimbal and receive response.

        Args:
            cmd_id: Command ID
            data: Command data payload
            need_ack: Wait for acknowledgment

        Returns:
            Response data dict or None if no response/error
        """
        if not self.sock:
            logger.error("No gimbal connection")
            return None

        packet = self._build_packet(cmd_id, data, need_ack)

        try:
            self.sock.sendto(packet, (self.ip, self.port))

            if need_ack:
                response, _addr = self.sock.recvfrom(1024)
                return self._parse_response(response)

        except socket.timeout:
            # Throttle timeout warnings to reduce log spam
            self._timeout_count += 1
            if self._timeout_count % self._timeout_warn_interval == 0:
                logger.warning(
                    f"Gimbal timeout (count: {self._timeout_count}) - "
                    f"This is normal if gimbal is busy or network is congested"
                )
            return None
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            return None

        return None

    # ===== Control Methods =====

    def set_attitude(
        self,
        yaw: Optional[float] = None,
        pitch: Optional[float] = None,
        skip_rate_limit: bool = False,
    ) -> bool:
        """
        Set gimbal attitude angles.

        Args:
            yaw: Target yaw angle in degrees (-135.0 to 135.0), None to keep current
            pitch: Target pitch angle in degrees (-90.0 to 25.0), None to keep current
            skip_rate_limit: Skip rate limiting (for manual commands)

        Returns:
            True if command successful
        """
        # Rate limiting: don't send commands faster than gimbal can handle
        if not skip_rate_limit:
            current_time = time.time()
            time_since_last = current_time - self._last_attitude_command
            if time_since_last < self._min_attitude_interval:
                # Too soon, skip this command
                return True  # Return True to avoid error spam
            self._last_attitude_command = current_time

        # Convert to int16 with 0.1 degree precision
        # -32768 means "no change"
        yaw_int = int(yaw * 10) if yaw is not None else -32768
        pitch_int = int(pitch * 10) if pitch is not None else -32768

        # Build data payload (yaw, pitch as int16 little-endian)
        data = struct.pack("<hh", yaw_int, pitch_int)

        # Send command without waiting for ACK (faster, non-blocking)
        # The gimbal will execute the command even without ACK
        response = self._send_command(self.CMD_SET_GIMBAL_ATTITUDE, data, need_ack=False)

        # Don't log failures for rate-limited commands
        if response or not skip_rate_limit:
            logger.debug(f"Set gimbal attitude: yaw={yaw}, pitch={pitch}")
            return True
        else:
            logger.warning("Failed to set gimbal attitude")
            return False

    def set_gimbal_rotation(self, yaw_speed: int = 0, pitch_speed: int = 0) -> bool:
        """
        Control gimbal rotation speed (CMD 0x07).

        This is the native velocity control command for continuous movement.

        Args:
            yaw_speed: Yaw rotation speed (-100 to +100, 0=stop)
            pitch_speed: Pitch rotation speed (-100 to +100, 0=stop)

        Returns:
            True if command successful
        """
        # Clamp speeds to valid range
        yaw_speed = max(-100, min(100, int(yaw_speed)))
        pitch_speed = max(-100, min(100, int(pitch_speed)))

        # Build data payload (yaw_speed, pitch_speed as int8)
        data = struct.pack("<bb", yaw_speed, pitch_speed)

        # Send command without waiting for ACK
        response = self._send_command(self.CMD_GIMBAL_ROTATION, data, need_ack=False)

        if response:
            logger.debug(f"Set gimbal rotation: yaw_speed={yaw_speed}, pitch_speed={pitch_speed}")
            return True
        return False

    def set_gimbal_velocity(self, yaw: float = 0.0, pitch: float = 0.0) -> bool:
        """
        Set gimbal velocity for smooth tracking.

        Uses the rotation command (0x07) for native velocity control.

        Args:
            yaw: Yaw rate in degrees/sec (positive = right, -30 to +30)
            pitch: Pitch rate in degrees/sec (positive = down, -30 to +30)

        Returns:
            True if command successful
        """
        # Convert degrees/sec to speed value (-100 to +100)
        # Max gimbal speed is approximately 30°/s, so normalize to that
        # speed = (degrees_per_sec / 30.0) * 100
        max_speed = 30.0  # degrees per second

        yaw_speed = int((yaw / max_speed) * 100)
        pitch_speed = int((pitch / max_speed) * 100)

        # Use the native rotation command
        return self.set_gimbal_rotation(yaw_speed=yaw_speed, pitch_speed=pitch_speed)


    def center(self, mode: int = 1) -> bool:
        """
        Center the gimbal to default position (yaw=0, pitch=0).

        Uses the dedicated center command (0x08).

        Args:
            mode: Center mode (1=center, 2=center downward, 3=center, 4=downward)
                  Default is 1 (one-key center to position 0)

        Returns:
            True if successful
        """
        logger.info("Centering gimbal...")

        # Temporarily increase timeout for center command
        old_timeout = self.sock.gettimeout() if self.sock else None
        if self.sock:
            self.sock.settimeout(1.0)  # 1 second for center

        try:
            # Pack the center mode as uint8_t
            data = struct.pack("<B", mode)
            response = self._send_command(self.CMD_CENTER_GIMBAL, data=data, need_ack=True)

            if response:
                logger.info("✓ Gimbal centered")
                return True
            else:
                logger.warning("Failed to center gimbal")
                return False
        finally:
            # Restore original timeout
            if self.sock and old_timeout is not None:
                self.sock.settimeout(old_timeout)

    def take_photo(self) -> bool:
        """
        Trigger photo capture.

        Returns:
            True if successful
        """
        logger.info("Taking photo...")
        response = self._send_command(self.CMD_PHOTO)
        return response is not None

    def toggle_recording(self) -> bool:
        """
        Toggle video recording on/off.

        Returns:
            True if successful
        """
        logger.info("Toggling recording...")
        response = self._send_command(self.CMD_RECORD)
        return response is not None

    def zoom(self, level: int) -> bool:
        """
        Set zoom level.

        Args:
            level: Zoom level (1 = zoom in, -1 = zoom out, 0 = stop)

        Returns:
            True if successful
        """
        data = struct.pack("<b", level)
        response = self._send_command(self.CMD_MANUAL_ZOOM, data)
        return response is not None

    def focus(self, direction: int) -> bool:
        """
        Manual focus control.

        Args:
            direction: Focus direction (1 = far, -1 = near, 0 = stop)

        Returns:
            True if successful
        """
        data = struct.pack("<b", direction)
        response = self._send_command(self.CMD_MANUAL_FOCUS, data)
        return response is not None

    def auto_focus(self) -> bool:
        """
        Trigger auto-focus.

        Returns:
            True if successful
        """
        logger.info("Triggering auto-focus...")
        response = self._send_command(self.CMD_AUTO_FOCUS)
        return response is not None

    # ===== Query Methods =====

    def get_firmware_version(self) -> Optional[str]:
        """
        Get gimbal firmware version.

        Returns:
            Version string or None if failed
        """
        response = self._send_command(self.CMD_ACQUIRE_FW_VER)

        if response and len(response["data"]) >= 12:
            version = response["data"][:12].decode("ascii", errors="ignore")
            logger.info(f"Gimbal firmware version: {version}")
            return version

        return None

    def get_attitude(self) -> Optional[Dict[str, float]]:
        """
        Get current gimbal attitude.

        Returns:
            Dict with yaw, pitch, roll in degrees, or None if failed
        """
        response = self._send_command(self.CMD_ACQUIRE_GIMBAL_INFO)

        if response and len(response["data"]) >= 6:
            yaw, pitch, roll = struct.unpack("<hhh", response["data"][:6])
            return {
                "yaw": yaw / 10.0,
                "pitch": pitch / 10.0,
                "roll": roll / 10.0,
            }

        return None

    def get_gimbal_info(self) -> Optional[Dict[str, float]]:
        """
        Alias for get_attitude() for backward compatibility.

        Returns:
            Dict with yaw, pitch, roll in degrees
        """
        return self.get_attitude()
