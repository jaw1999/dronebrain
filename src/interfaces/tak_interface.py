"""
TAK Server Interface

Manages connection and communication with Team Awareness Kit (TAK) servers.
Sends CoT (Cursor on Target) messages over TCP or UDP.
"""

import logging
import socket
import threading
import time
from typing import Optional
from queue import Queue, Empty

logger = logging.getLogger(__name__)


class TAKInterface:
    """
    Interface for sending CoT messages to TAK servers.

    Supports both TCP and UDP connections. TCP is recommended for
    reliable delivery.

    Example:
        >>> tak = TAKInterface(
        ...     server_ip="192.168.1.100",
        ...     server_port=8087,
        ...     protocol="tcp"
        ... )
        >>> tak.connect()
        >>> tak.send_cot(cot_xml_message)
        >>> tak.disconnect()
    """

    def __init__(
        self,
        server_ip: str,
        server_port: int = 8087,
        protocol: str = "tcp",
        reconnect_interval: float = 5.0,
        queue_size: int = 100,
    ):
        """
        Initialize TAK server interface.

        Args:
            server_ip: TAK server IP address
            server_port: TAK server port (default 8087 for TCP CoT)
            protocol: "tcp" or "udp"
            reconnect_interval: Seconds between reconnection attempts
            queue_size: Maximum number of queued messages
        """
        self.server_ip = server_ip
        self.server_port = server_port
        self.protocol = protocol.lower()
        self.reconnect_interval = reconnect_interval

        if self.protocol not in ("tcp", "udp"):
            raise ValueError(f"Protocol must be 'tcp' or 'udp', got: {protocol}")

        # Connection state
        self.sock: Optional[socket.socket] = None
        self.connected = False
        self._running = False
        self._send_thread: Optional[threading.Thread] = None
        self._message_queue: Queue = Queue(maxsize=queue_size)
        self._stats = {
            "messages_sent": 0,
            "messages_failed": 0,
            "bytes_sent": 0,
        }

    def connect(self) -> bool:
        """
        Connect to TAK server.

        Returns:
            True if connection successful
        """
        try:
            if self.protocol == "tcp":
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5.0)
                self.sock.connect((self.server_ip, self.server_port))
                logger.info(f"Connected to TAK server at {self.server_ip}:{self.server_port} (TCP)")
            else:  # UDP
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                logger.info(
                    f"Configured for TAK server at {self.server_ip}:{self.server_port} (UDP)"
                )

            self.connected = True
            return True

        except Exception as e:
            logger.error(f"Failed to connect to TAK server: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Disconnect from TAK server."""
        if self.sock:
            try:
                self.sock.close()
            except Exception as e:
                logger.error(f"Error closing socket: {e}")
            finally:
                self.sock = None
                self.connected = False
                logger.info("Disconnected from TAK server")

    def start(self) -> bool:
        """
        Start background thread for sending messages.

        Returns:
            True if started successfully
        """
        if self._running:
            logger.warning("TAK interface already running")
            return True

        if not self.connect():
            return False

        self._running = True
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._send_thread.start()
        logger.info("TAK interface started")
        return True

    def stop(self):
        """Stop background thread and disconnect."""
        if not self._running:
            return

        self._running = False

        # Wait for thread to finish
        if self._send_thread and self._send_thread.is_alive():
            self._send_thread.join(timeout=2.0)

        self.disconnect()
        logger.info("TAK interface stopped")

    def send_cot(self, cot_message: str) -> bool:
        """
        Queue a CoT message for sending to TAK server.

        Args:
            cot_message: CoT XML message string

        Returns:
            True if message was queued successfully
        """
        try:
            self._message_queue.put_nowait(cot_message)
            return True
        except Exception as e:
            logger.warning(f"Failed to queue CoT message: {e}")
            self._stats["messages_failed"] += 1
            return False

    def send_cot_immediate(self, cot_message: str) -> bool:
        """
        Send a CoT message immediately (blocking).

        Args:
            cot_message: CoT XML message string

        Returns:
            True if message was sent successfully
        """
        if not self.connected:
            logger.warning("Not connected to TAK server")
            return False

        try:
            message_bytes = cot_message.encode("utf-8")

            if self.protocol == "tcp":
                # TCP: send with message terminator
                self.sock.sendall(message_bytes)
            else:  # UDP
                self.sock.sendto(message_bytes, (self.server_ip, self.server_port))

            self._stats["messages_sent"] += 1
            self._stats["bytes_sent"] += len(message_bytes)
            logger.debug(f"Sent CoT message ({len(message_bytes)} bytes)")
            return True

        except Exception as e:
            logger.error(f"Failed to send CoT message: {e}")
            self._stats["messages_failed"] += 1
            self.connected = False
            return False

    def _send_loop(self):
        """Background thread for sending queued messages."""
        logger.info("TAK send loop started")

        while self._running:
            try:
                # Try to reconnect if disconnected
                if not self.connected:
                    logger.info("Attempting to reconnect to TAK server...")
                    if self.connect():
                        logger.info("Reconnected to TAK server")
                    else:
                        time.sleep(self.reconnect_interval)
                        continue

                # Get message from queue (with timeout)
                try:
                    cot_message = self._message_queue.get(timeout=0.5)
                except Empty:
                    continue

                # Send message
                if not self.send_cot_immediate(cot_message):
                    # Failed to send - will try to reconnect on next iteration
                    logger.warning("Failed to send message, will retry connection")

            except Exception as e:
                logger.error(f"Error in TAK send loop: {e}")
                time.sleep(1.0)

        logger.info("TAK send loop stopped")

    def get_stats(self) -> dict:
        """
        Get interface statistics.

        Returns:
            Dictionary with statistics
        """
        return {
            "connected": self.connected,
            "queue_size": self._message_queue.qsize(),
            "messages_sent": self._stats["messages_sent"],
            "messages_failed": self._stats["messages_failed"],
            "bytes_sent": self._stats["bytes_sent"],
        }

    def is_connected(self) -> bool:
        """Check if connected to TAK server."""
        return self.connected
