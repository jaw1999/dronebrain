"""
Pipeline Manager - Central Orchestrator for DroneBrain.

This module coordinates all system components including:
- Hardware interfaces (MAVLink, Siyi gimbal, RTSP capture)
- Vision processing pipeline
- Target tracking
- Autonomous behaviors
- Streaming and TAK integration

The PipelineManager handles initialization, shutdown, and lifecycle
management of all subsystems.

Author: DroneBrain Team
"""

import logging
import math
import signal
import threading
import time
from pathlib import Path
from typing import Optional, Dict, Any

import yaml

from .state import SystemState, SystemStatus
from ..interfaces.mavlink_interface import MAVLinkInterface
from ..interfaces.siyi_interface import SiyiInterface
from ..interfaces.rtsp_capture import RTSPCapture, RTSPCaptureGStreamer

logger = logging.getLogger(__name__)


class PipelineManager:
    """
    Central orchestrator for DroneBrain system.

    Manages initialization, coordination, and shutdown of all subsystems.

    Attributes:
        state: Global system state
        config: System configuration
        mavlink: MAVLink interface
        siyi: Siyi gimbal interface
        camera: RTSP video capture
        running: System running flag

    Example:
        >>> manager = PipelineManager("config/config.yaml")
        >>> manager.initialize()
        >>> manager.start()
        >>> # ... system runs ...
        >>> manager.shutdown()
    """

    def __init__(self, config_path: str = "config/config.yaml"):
        """
        Initialize pipeline manager.

        Args:
            config_path: Path to configuration file
        """
        self.config_path = config_path
        self.config: Dict[str, Any] = {}

        # Global state
        self.state = SystemState()

        # Component interfaces
        self.mavlink: Optional[MAVLinkInterface] = None
        self.siyi: Optional[SiyiInterface] = None
        self.camera: Optional[RTSPCapture] = None

        # Vision pipeline (Phase 2)
        self.vision_detector = None
        self.geolocator = None  # Phase 3: Geolocation
        self._vision_thread: Optional[threading.Thread] = None
        self._annotated_frame: Optional[Any] = None
        self._annotated_frame_lock = threading.Lock()

        # Streaming (Phase 2)
        self.raw_stream = None
        self.annotated_stream = None

        # Autonomous tracking (Phase 4)
        self.tracker = None
        self._tracking_enabled = False
        self._last_tracking_update = 0.0

        # Autonomous flight behaviors
        self.autonomous_manager = None

        # TAK integration (Phase 5)
        self.tak_client = None
        self.cot_generator = None
        self._tak_publish_interval = 1.0  # Publish CoT every 1 second
        self._last_tak_publish = 0.0

        # Web UI (Phase 6)
        self.web_server = None

        # System control
        self.running = False
        self._shutdown_requested = False

        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, sig, frame):
        """
        Handle shutdown signals (SIGINT, SIGTERM).

        Args:
            sig: Signal number
            frame: Current stack frame
        """
        logger.info(f"Received signal {sig}, initiating shutdown...")
        self._shutdown_requested = True

    def load_config(self) -> bool:
        """
        Load system configuration from YAML file.

        Returns:
            True if successful
        """
        try:
            config_file = Path(self.config_path)

            if not config_file.exists():
                logger.error(f"Config file not found: {self.config_path}")
                logger.info("Please copy config.example.yaml to config.yaml")
                return False

            with open(config_file, "r") as f:
                self.config = yaml.safe_load(f)

            # Store in state for global access
            self.state.set_config(self.config)

            logger.info(f"Configuration loaded from {self.config_path}")
            return True

        except Exception as e:
            logger.error(f"Error loading configuration: {e}")
            return False

    def initialize(self) -> bool:
        """
        Initialize all system components.

        Returns:
            True if all components initialized successfully
        """
        logger.info("=" * 60)
        logger.info("Initializing DroneBrain System")
        logger.info("=" * 60)

        self.state.set_status(SystemStatus.INITIALIZING)

        # Load configuration
        if not self.load_config():
            logger.error("Failed to load configuration")
            self.state.set_status(SystemStatus.ERROR)
            return False

        # Initialize MAVLink interface
        logger.info("\n[1/3] Initializing MAVLink interface...")
        if not self._init_mavlink():
            logger.error("MAVLink initialization failed")
            # Continue anyway - might work without drone connection
            # self.state.set_status(SystemStatus.ERROR)
            # return False

        # Initialize Siyi gimbal interface
        logger.info("\n[2/3] Initializing Siyi gimbal interface...")
        if not self._init_siyi():
            logger.error("Siyi initialization failed")
            # Continue anyway - might work without gimbal
            # self.state.set_status(SystemStatus.ERROR)
            # return False

        # Initialize camera capture
        logger.info("\n[3/4] Initializing camera capture...")
        if not self._init_camera():
            logger.error("Camera initialization failed")
            # Continue anyway for testing
            # self.state.set_status(SystemStatus.ERROR)
            # return False

        # Initialize vision pipeline (Phase 2)
        logger.info("\n[4/4] Initializing vision pipeline...")
        if not self._init_vision():
            logger.warning("Vision detector initialization failed")
            # Continue anyway - vision is optional

        # Initialize streaming (Phase 2)
        logger.info("\n[5/5] Initializing video streaming...")
        if not self._init_streaming():
            logger.warning("Streaming initialization failed")
            # Continue anyway - streaming is optional

        # Initialize autonomous behaviors
        logger.info("\n[6/8] Initializing autonomous flight manager...")
        self._init_autonomous()

        # Initialize TAK integration (Phase 5)
        logger.info("\n[7/8] Initializing TAK integration...")
        self._init_tak()

        # Initialize web UI (Phase 6)
        logger.info("\n[8/8] Initializing web interface...")
        self._init_web_ui()

        logger.info("\n" + "=" * 60)
        logger.info("DroneBrain System Initialized")
        logger.info("=" * 60)

        self.state.set_status(SystemStatus.READY)
        return True

    def _init_mavlink(self) -> bool:
        """
        Initialize MAVLink interface.

        Returns:
            True if successful
        """
        try:
            mavlink_config = self.config.get("drone", {})
            ip = mavlink_config.get("mavlink_ip", "192.168.144.10")
            port = mavlink_config.get("mavlink_port", 14550)
            protocol = mavlink_config.get("mavlink_protocol", "udpout")
            sys_id = mavlink_config.get("system_id", 255)
            comp_id = mavlink_config.get("component_id", 190)

            self.mavlink = MAVLinkInterface(
                state=self.state,
                ip=ip,
                port=port,
                protocol=protocol,
                system_id=sys_id,
                component_id=comp_id,
            )

            if self.mavlink.connect():
                logger.info("✓ MAVLink interface initialized")
                return True
            else:
                logger.warning("✗ MAVLink connection failed")
                return False

        except Exception as e:
            logger.error(f"Error initializing MAVLink: {e}")
            self.state.add_error(f"MAVLink init error: {e}")
            return False

    def _init_siyi(self) -> bool:
        """
        Initialize Siyi gimbal interface.

        Returns:
            True if successful
        """
        try:
            camera_config = self.config.get("camera", {})
            ip = camera_config.get("siyi_ip", "192.168.144.25")
            port = camera_config.get("siyi_port", 37260)

            self.siyi = SiyiInterface(
                state=self.state,
                ip=ip,
                port=port,
            )

            if self.siyi.connect():
                logger.info("✓ Siyi gimbal interface initialized")
                return True
            else:
                logger.warning("✗ Siyi connection failed")
                return False

        except Exception as e:
            logger.error(f"Error initializing Siyi: {e}")
            self.state.add_error(f"Siyi init error: {e}")
            return False

    def _init_camera(self) -> bool:
        """
        Initialize camera RTSP capture.

        Returns:
            True if successful
        """
        try:
            camera_config = self.config.get("camera", {})
            rtsp_url = camera_config.get("rtsp_url", "rtsp://192.168.144.25:8554/main.264")
            transport = camera_config.get("rtsp_transport", "tcp")

            # Try GStreamer first (better for Jetson), fallback to FFmpeg
            try:
                self.camera = RTSPCaptureGStreamer(
                    rtsp_url=rtsp_url,
                    transport=transport,
                )
                logger.info("Using GStreamer backend for video capture")
            except Exception as e:
                logger.debug(f"GStreamer unavailable ({e}), falling back to FFmpeg")
                self.camera = RTSPCapture(
                    rtsp_url=rtsp_url,
                    transport=transport,
                )
                logger.info("Using FFmpeg backend for video capture")

            # Mark camera as connected (resolution will be updated after first frame)
            self.state.update_camera_state(connected=True)

            logger.info("✓ Camera capture initialized")
            return True

        except Exception as e:
            logger.error(f"Error initializing camera: {e}")
            self.state.add_error(f"Camera init error: {e}")
            self.state.update_camera_state(connected=False)
            return False

    def _init_vision(self) -> bool:
        """
        Initialize vision detection pipeline.

        Returns:
            True if successful
        """
        try:
            # Import here to avoid circular dependency
            from ..tracking.geolocator import Geolocator, CameraIntrinsics

            vision_config = self.config.get("vision", {})
            detector_config = vision_config.get("detector", {})

            # Initialize YOLO detector
            logger.info("Initializing YOLO detector...")
            from ..vision.yolo_detector import YOLODetector

            self.vision_detector = YOLODetector(
                state=self.state,
                model_size=detector_config.get("model_size", "n"),
                confidence_threshold=detector_config.get("confidence_threshold", 0.30),
                classes=detector_config.get("classes", None),
            )
            logger.info("✓ YOLO detector initialized")

            # Initialize geolocator with camera intrinsics (Phase 3)
            camera_config = self.config.get("camera", {})
            intrinsics_config = camera_config.get("intrinsics", {})

            camera_intrinsics = CameraIntrinsics(
                focal_length_px=intrinsics_config.get("focal_length_px", 1280.0),
                cx=intrinsics_config.get("cx", 640.0),
                cy=intrinsics_config.get("cy", 360.0),
                sensor_width_px=intrinsics_config.get("sensor_width_px", 1280),
                sensor_height_px=intrinsics_config.get("sensor_height_px", 720),
            )

            self.geolocator = Geolocator(camera_intrinsics)
            logger.info("✓ Geolocator initialized")

            # Initialize target tracker (Phase 4)
            from ..control.target_tracker import TargetTracker

            tracking_config = vision_config.get("tracking", {})
            self.tracker = TargetTracker(
                camera_width=intrinsics_config.get("sensor_width_px", 1280),
                camera_height=intrinsics_config.get("sensor_height_px", 720),
                pid_yaw_gains=tracking_config.get("pid_yaw_gains", (1.5, 0.0, 0.1)),
                pid_pitch_gains=tracking_config.get("pid_pitch_gains", (1.5, 0.0, 0.1)),
                deadzone_px=tracking_config.get("deadzone_px", 30),
                max_gimbal_speed=tracking_config.get("max_gimbal_speed", 30.0),
                lost_timeout=tracking_config.get("lost_timeout", 2.0),
            )
            logger.info("✓ Target tracker initialized")

            logger.info("✓ Vision detector initialized")
            return True

        except Exception as e:
            logger.error(f"Error initializing vision detector: {e}")
            self.state.add_error(f"Vision init error: {e}")
            return False

    def _init_streaming(self) -> bool:
        """
        Initialize video streaming publishers.

        Returns:
            True if successful
        """
        try:
            # Import here to avoid issues
            from ..streaming.stream_publisher import StreamPublisher

            # Get camera resolution from camera stats if available,
            # otherwise use defaults from config
            width = 1280
            height = 720
            fps = 25

            if self.camera:
                stats = self.camera.get_stats()
                if stats.get("opened"):
                    # Parse resolution string like "1280x720"
                    resolution = stats.get("resolution", "1280x720")
                    if "x" in resolution:
                        w, h = resolution.split("x")
                        width = int(w)
                        height = int(h)
                    fps = int(stats.get("fps", 25))

            # Get streaming config
            vision_config = self.config.get("vision", {})
            streaming_config = vision_config.get("streaming", {})

            raw_fps = streaming_config.get("raw_fps", fps)
            raw_bitrate = streaming_config.get("raw_bitrate", "2M")

            annotated_scale = streaming_config.get("annotated_resolution_scale", 1.0)
            annotated_fps = streaming_config.get("annotated_fps", fps)
            annotated_bitrate = streaming_config.get("annotated_bitrate", "2M")

            # Calculate annotated resolution
            annotated_width = int(width * annotated_scale)
            annotated_height = int(height * annotated_scale)

            # Initialize raw video stream publisher
            logger.info("  Initializing raw video stream...")
            self.raw_stream = StreamPublisher(
                stream_name="raw",
                width=width,
                height=height,
                fps=raw_fps,
                bitrate=raw_bitrate,
            )
            logger.info(f"  ✓ Raw stream: {width}x{height} @ {raw_fps}fps, {raw_bitrate}")

            # Initialize annotated video stream publisher (lower resolution/fps for performance)
            logger.info("  Initializing annotated video stream...")
            self.annotated_stream = StreamPublisher(
                stream_name="annotated",
                width=annotated_width,
                height=annotated_height,
                fps=annotated_fps,
                bitrate=annotated_bitrate,
            )
            logger.info(
                f"  ✓ Annotated stream: {annotated_width}x{annotated_height} "
                f"@ {annotated_fps}fps, {annotated_bitrate}"
            )

            logger.info("✓ Video streaming initialized")
            logger.info("  Stream URLs:")
            logger.info("    Raw:       rtsp://localhost:8554/raw")
            logger.info("    Annotated: rtsp://localhost:8554/annotated")
            logger.info(
                "    HLS (web): http://localhost:8888/raw / " "http://localhost:8888/annotated"
            )

            return True

        except Exception as e:
            logger.error(f"Error initializing streaming: {e}")
            self.state.add_error(f"Streaming init error: {e}")
            return False

    def _init_autonomous(self) -> bool:
        """
        Initialize autonomous flight manager.

        Returns:
            True if successful
        """
        try:
            from ..autonomous.autonomous_manager import AutonomousManager

            autonomous_config = self.config.get("autonomous", {})

            # Require MAVLink for autonomous flight
            if not self.mavlink:
                logger.warning("  Autonomous manager requires MAVLink - skipping")
                return False

            # Initialize autonomous manager
            self.autonomous_manager = AutonomousManager(
                state=self.state,
                mavlink=self.mavlink,
                config=autonomous_config,
            )

            logger.info(
                f"  ✓ Autonomous manager initialized "
                f"(standoff: {autonomous_config.get('standoff_distance', 50.0)}m)"
            )
            logger.info("✓ Autonomous flight manager ready")
            return True

        except Exception as e:
            logger.error(f"Error initializing autonomous manager: {e}")
            self.state.add_error(f"Autonomous init error: {e}")
            # Autonomous is optional, don't fail initialization
            return False

    def _init_tak(self) -> bool:
        """
        Initialize TAK (Team Awareness Kit) integration.

        Returns:
            True if successful
        """
        try:
            # Import TAK modules
            from ..interfaces.tak_interface import TAKInterface
            from ..interfaces.cot_generator import CoTGenerator

            tak_config = self.config.get("tak", {})

            # Skip if TAK is disabled
            if not tak_config.get("enabled", False):
                logger.info("  TAK integration disabled in config")
                return True

            server_ip = tak_config.get("server_ip", "127.0.0.1")
            server_port = tak_config.get("server_port", 8087)
            protocol = tak_config.get("protocol", "tcp")
            callsign = tak_config.get("callsign", "DRONEBRAIN")

            # Initialize CoT message generator
            self.cot_generator = CoTGenerator(
                source_callsign=callsign,
                stale_minutes=tak_config.get("stale_minutes", 5),
                default_ce=tak_config.get("default_ce", 10.0),
                default_le=tak_config.get("default_le", 10.0),
            )
            logger.info(f"  ✓ CoT generator initialized (callsign: {callsign})")

            # Initialize TAK server interface
            self.tak_client = TAKInterface(
                server_ip=server_ip,
                server_port=server_port,
                protocol=protocol,
                reconnect_interval=tak_config.get("reconnect_interval", 5.0),
                queue_size=tak_config.get("queue_size", 100),
            )
            logger.info(
                f"  ✓ TAK interface configured " f"({protocol.upper()}://{server_ip}:{server_port})"
            )

            # Get publish interval
            self._tak_publish_interval = tak_config.get("publish_interval", 1.0)

            logger.info("✓ TAK integration initialized")
            return True

        except Exception as e:
            logger.error(f"Error initializing TAK integration: {e}")
            self.state.add_error(f"TAK init error: {e}")
            # TAK is optional, don't fail initialization
            return False

    def _init_web_ui(self) -> bool:
        """
        Initialize Web UI server.

        Returns:
            True if successful
        """
        try:
            # Import web server
            from ..web.web_server import WebServer

            web_config = self.config.get("web", {})

            # Skip if web UI is disabled
            if not web_config.get("enabled", True):  # Default enabled
                logger.info("  Web UI disabled in config")
                return True

            host = web_config.get("host", "0.0.0.0")
            port = web_config.get("port", 8080)

            # Initialize web server
            self.web_server = WebServer(
                pipeline=self,
                host=host,
                port=port,
            )
            logger.info(f"  ✓ Web UI configured (http://{host}:{port})")

            logger.info("✓ Web UI initialized")
            return True

        except Exception as e:
            logger.error(f"Error initializing web UI: {e}")
            self.state.add_error(f"Web UI init error: {e}")
            # Web UI is optional, don't fail initialization
            return False

    def _vision_processing_loop(self):
        """
        Vision processing loop (runs in background thread).

        Continuously gets frames from camera, runs detection, and stores
        annotated frames for streaming.
        """
        logger.info("Vision processing loop started")

        frame_count = 0
        last_log_time = time.time()
        last_annotated_publish_time = 0.0

        # Get annotated stream target FPS for rate limiting
        vision_config = self.config.get("vision", {})
        streaming_config = vision_config.get("streaming", {})
        annotated_fps = streaming_config.get("annotated_fps", 25)
        annotated_frame_interval = 1.0 / annotated_fps if annotated_fps > 0 else 0.0

        while self.running:
            try:
                if not self.camera or not self.vision_detector:
                    time.sleep(0.1)
                    continue

                # Get latest frame from camera
                frame = self.camera.get_frame(timeout=0.5)
                if frame is None:
                    continue

                # Update camera resolution from actual frame dimensions (once)
                camera_state = self.state.get_camera_state()
                if camera_state.resolution == "Unknown" and frame is not None:
                    height, width = frame.shape[:2]
                    self.state.update_camera_state(resolution=f"{width}x{height}")

                # Increment frame counter and update FPS
                self.state.increment_frame_count()

                # Publish raw frame to raw stream
                if self.raw_stream and self.raw_stream.is_running():
                    self.raw_stream.publish_frame(frame)

                # Run detection (always run to maintain tracker state)
                timestamp = time.time()
                annotated_frame, detections = self.vision_detector.detect(
                    frame, timestamp=timestamp
                )

                # Check if we should publish annotated frame (rate limiting)
                should_publish_annotated = (
                    timestamp - last_annotated_publish_time >= annotated_frame_interval
                )

                # Calculate GPS coordinates for all detections
                if self.geolocator and detections:
                    drone_state = self.state.get_drone_state()
                    gimbal_state = self.state.get_gimbal_state()

                    # Log telemetry state when we have detections
                    logger.info(
                        f"GPS calc check: drone_connected={drone_state.connected}, "
                        f"drone_lat={drone_state.latitude:.6f}, drone_lon={drone_state.longitude:.6f}, "
                        f"drone_heading={drone_state.heading:.1f}°, "
                        f"gimbal_connected={gimbal_state.connected}, gimbal_pitch={gimbal_state.pitch:.1f}°, "
                        f"gimbal_yaw={gimbal_state.yaw:.1f}°"
                    )

                    # Only calculate if we have valid drone GPS
                    # Gimbal connection not required - will use last known gimbal angles
                    if (
                        drone_state.connected
                        and drone_state.latitude != 0.0
                    ):
                        # Check for test altitude override (for ground testing photogrammetry)
                        test_alt_override = self.config.get("camera", {}).get("test_altitude_override")

                        if test_alt_override is not None:
                            # Use override altitude for testing
                            effective_altitude = float(test_alt_override)
                            logger.debug(f"Using test altitude override: {effective_altitude:.1f}m (actual: {drone_state.altitude_relative:.1f}m)")
                        else:
                            # Use actual altitude with minimum threshold
                            effective_altitude = max(abs(drone_state.altitude_relative), 0.5)

                        # Convert Siyi gimbal pitch to standard convention
                        # Siyi: 0°=down, +90°=forward. Standard: 0°=forward, -90°=down
                        gimbal_pitch_converted = gimbal_state.pitch - 90.0

                        for detection in detections:
                            # Calculate object center in pixels
                            x1, y1, x2, y2 = detection.bbox
                            center_x = (x1 + x2) / 2
                            center_y = (y1 + y2) / 2

                            # Calculate GPS coordinates
                            # Note: Adding 180° to heading to correct for coordinate system mismatch
                            corrected_heading = (drone_state.heading + 180.0) % 360.0
                            gps_coords = self.geolocator.pixel_to_gps(
                                pixel_x=center_x,
                                pixel_y=center_y,
                                drone_lat=drone_state.latitude,
                                drone_lon=drone_state.longitude,
                                drone_alt=effective_altitude,
                                drone_roll=drone_state.roll,
                                drone_pitch=drone_state.pitch,
                                drone_yaw=corrected_heading,
                                gimbal_pitch=gimbal_pitch_converted,
                                gimbal_yaw=gimbal_state.yaw,
                                ground_elevation=0.0,
                            )

                            if gps_coords:
                                detection.latitude, detection.longitude = gps_coords

                                # Calculate accurate slant range using photogrammetry
                                # This accounts for the actual bearing angle to the target
                                x_norm = (center_x - self.geolocator.intrinsics.cx) / self.geolocator.intrinsics.focal_length_px
                                y_norm = (center_y - self.geolocator.intrinsics.cy) / self.geolocator.intrinsics.focal_length_px
                                bearing_y = math.atan(y_norm)  # Vertical bearing angle

                                slant_range = self.geolocator.calculate_slant_range(
                                    drone_alt=effective_altitude,
                                    drone_pitch=drone_state.pitch,
                                    gimbal_pitch=gimbal_pitch_converted,
                                    bearing_y=bearing_y,
                                    ground_elevation=0.0
                                )

                                # Calculate GPS-based horizontal distance for comparison
                                # Haversine formula for great circle distance
                                lat1_rad = math.radians(drone_state.latitude)
                                lat2_rad = math.radians(detection.latitude)
                                dLat = lat2_rad - lat1_rad
                                dLon = math.radians(detection.longitude - drone_state.longitude)

                                a = math.sin(dLat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dLon/2)**2
                                c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
                                gps_horiz_dist = 6371000 * c  # Earth radius in meters

                                # 3D distance including altitude
                                gps_3d_dist = math.sqrt(gps_horiz_dist**2 + effective_altitude**2)

                                if slant_range:
                                    detection.distance = slant_range

                                dist_str = f"{detection.distance:.1f}m" if detection.distance else "N/A"
                                logger.info(
                                    f"GPS calc for {detection.class_name} (track {detection.track_id}): "
                                    f"lat={detection.latitude:.6f}, lon={detection.longitude:.6f}, "
                                    f"slant={dist_str}, gps_dist={gps_3d_dist:.1f}m, alt={effective_altitude:.1f}m, "
                                    f"pixel_y={center_y:.0f}, bearing_y={math.degrees(bearing_y):.1f}°, "
                                    f"gimbal={gimbal_state.pitch:.1f}° (conv={gimbal_pitch_converted:.1f}°)"
                                )
                            else:
                                logger.info(
                                    f"GPS calc failed for {detection.class_name} - "
                                    f"gimbal_pitch={gimbal_state.pitch:.1f}° (raw), "
                                    f"{gimbal_pitch_converted:.1f}° (converted), "
                                    f"drone_pitch={drone_state.pitch:.1f}°"
                                )

                # Update state with detections (including GPS data)
                if detections:
                    self.state.update_detections(detections)

                # Publish tracked target to TAK server (Phase 5) - Optimized
                if (
                    self.tak_client
                    and self.cot_generator
                    and detections
                    and (timestamp - self._last_tak_publish >= self._tak_publish_interval)
                ):
                    # Only publish if TAK client is running
                    if self.tak_client.is_connected():
                        for detection in detections:
                            # Only publish the actively tracked object with GPS coordinates
                            if (
                                detection.track_id is not None
                                and hasattr(detection, "gps_lat")
                                and detection.gps_lat is not None
                            ):
                                # Generate CoT message for this detection
                                cot_message = self.cot_generator.generate_detection_cot(
                                    detection=detection,
                                    lat=detection.gps_lat,
                                    lon=detection.gps_lon,
                                    alt_msl=detection.gps_alt,
                                )

                                # Queue message for sending
                                self.tak_client.send_cot(cot_message)
                                logger.debug(
                                    f"Published CoT for Track ID {detection.track_id} "
                                    f"({detection.class_name}) to TAK server"
                                )
                                break  # Only publish one target per interval

                        self._last_tak_publish = timestamp

                # Update autonomous tracker and execute gimbal commands (Phase 4)
                if self.tracker and self._tracking_enabled and self.siyi:
                    # Calculate time since last tracking update
                    tracking_dt = (
                        timestamp - self._last_tracking_update
                        if self._last_tracking_update > 0
                        else 0.033
                    )
                    self._last_tracking_update = timestamp

                    # Update tracker with current detections
                    yaw_rate, pitch_rate = self.tracker.update(detections, dt=tracking_dt)

                    # Log tracking state
                    target = self.tracker.get_target()
                    state = self.tracker.get_state()
                    if target:
                        logger.info(
                            f"Tracker: {state.value} | ID:{target.track_id} "
                            f"({target.class_name}) | "
                            f"Pos:({target.center_x:.0f},{target.center_y:.0f}) | "
                            f"Cmd: Yaw={yaw_rate:.2f}°/s, Pitch={pitch_rate:.2f}°/s"
                        )

                    # Send gimbal commands if tracker is actively tracking
                    if self.tracker.is_tracking():
                        # Always send commands when tracking, even if small
                        # The rate limiter in siyi_interface will prevent flooding
                        self.siyi.set_gimbal_velocity(yaw=yaw_rate, pitch=pitch_rate)

                # Update autonomous flight manager
                if self.autonomous_manager:
                    self.autonomous_manager.update(detections)

                # Store annotated frame for streaming
                with self._annotated_frame_lock:
                    self._annotated_frame = annotated_frame

                # Publish annotated frame to annotated stream (rate limited)
                if should_publish_annotated:
                    if self.annotated_stream and self.annotated_stream.is_running():
                        self.annotated_stream.publish_frame(annotated_frame)
                        last_annotated_publish_time = timestamp

                # Log stats every 5 seconds
                frame_count += 1
                if time.time() - last_log_time >= 5.0:
                    fps = frame_count / (time.time() - last_log_time)
                    logger.debug(
                        f"Vision processing: {fps:.1f} FPS, "
                        f"{len(detections)} detections in last frame"
                    )
                    frame_count = 0
                    last_log_time = time.time()

            except Exception as e:
                logger.error(f"Error in vision processing loop: {e}")
                time.sleep(0.1)

        logger.info("Vision processing loop stopped")

    def start(self):
        """Start all system components."""
        if self.running:
            logger.warning("System already running")
            return

        logger.info("\n" + "=" * 60)
        logger.info("Starting DroneBrain System")
        logger.info("=" * 60)

        self.running = True
        self.state.set_status(SystemStatus.ACTIVE)

        # Start MAVLink telemetry
        if self.mavlink:
            logger.info("[1/3] Starting MAVLink telemetry...")
            self.mavlink.start()

        # Start Siyi telemetry
        if self.siyi:
            logger.info("[2/3] Starting Siyi gimbal telemetry...")
            self.siyi.start()

        # Start camera capture
        if self.camera:
            logger.info("[3/4] Starting camera capture...")
            self.camera.start()

        # Start vision processing (Phase 2)
        if self.vision_detector and self.camera:
            logger.info("[4/4] Starting vision processing...")
            self._vision_thread = threading.Thread(target=self._vision_processing_loop, daemon=True)
            self._vision_thread.start()
            logger.info("Vision processing thread started")

        # Start streaming (Phase 2)
        if self.raw_stream:
            logger.info("[5/7] Starting raw video stream...")
            self.raw_stream.start()
        if self.annotated_stream:
            logger.info("[6/7] Starting annotated video stream...")
            self.annotated_stream.start()

        # Start TAK integration (Phase 5)
        if self.tak_client:
            logger.info("[7/8] Starting TAK integration...")
            self.tak_client.start()
            logger.info("TAK client started")

        # Start web UI (Phase 6)
        if self.web_server:
            logger.info("[8/8] Starting web interface...")
            self.web_server.start()
            logger.info("Web UI started")

        logger.info("\n" + "=" * 60)
        logger.info("DroneBrain System Running")
        logger.info("=" * 60)

        self._print_status()

    def run(self):
        """
        Main run loop.

        Keeps the system running and monitors for shutdown requests.
        """
        logger.info("\nSystem operational. Press Ctrl+C to shutdown.")

        try:
            while self.running and not self._shutdown_requested:
                # Print status periodically
                time.sleep(5.0)
                self._print_status()

        except KeyboardInterrupt:
            logger.info("\nKeyboard interrupt received")

        finally:
            self.shutdown()

    def shutdown(self):
        """Gracefully shutdown all system components."""
        if not self.running:
            return

        logger.info("\n" + "=" * 60)
        logger.info("Shutting Down DroneBrain System")
        logger.info("=" * 60)

        self.running = False
        self.state.set_status(SystemStatus.SHUTDOWN)

        # Stop web UI first
        if self.web_server:
            logger.info("[1/8] Stopping web interface...")
            self.web_server.stop()

        # Stop TAK integration
        if self.tak_client:
            logger.info("[2/8] Stopping TAK integration...")
            self.tak_client.stop()

        # Stop streaming
        if self.annotated_stream:
            logger.info("[3/6] Stopping annotated video stream...")
            self.annotated_stream.stop()
        if self.raw_stream:
            logger.info("[4/6] Stopping raw video stream...")
            self.raw_stream.stop()

        # Stop vision processing
        if self._vision_thread:
            logger.info("[1/4] Stopping vision processing...")
            # Thread will stop when self.running becomes False
            if self._vision_thread.is_alive():
                self._vision_thread.join(timeout=2.0)

        # Stop camera capture
        if self.camera:
            logger.info("[2/4] Stopping camera capture...")
            self.camera.stop()

        # Stop Siyi gimbal
        if self.siyi:
            logger.info("[2/3] Stopping Siyi gimbal interface...")
            self.siyi.stop()

        # Stop MAVLink
        if self.mavlink:
            logger.info("[3/3] Stopping MAVLink interface...")
            self.mavlink.stop()

        logger.info("\n" + "=" * 60)
        logger.info("DroneBrain System Shutdown Complete")
        logger.info("=" * 60)

    def _print_status(self):
        """Print current system status."""
        summary = self.state.get_summary()

        logger.info("\n" + "-" * 60)
        logger.info("SYSTEM STATUS")
        logger.info("-" * 60)

        # Overall status
        logger.info(f"Status: {summary['status'].value.upper()}")

        # Drone status
        drone = summary["drone"]
        logger.info("\nDrone:")
        logger.info(f"  Connected: {drone['connected']}")
        if drone["connected"]:
            logger.info(f"  Armed: {drone['armed']}")
            logger.info(f"  Mode: {drone['mode']}")
            logger.info(
                f"  Position: {drone['latitude']:.6f}, "
                f"{drone['longitude']:.6f}, "
                f"{drone['altitude_relative']:.1f}m"
            )
            logger.info(f"  Battery: {drone['battery_percent']:.0f}%")

        # Gimbal status
        gimbal = summary["gimbal"]
        logger.info("\nGimbal:")
        logger.info(f"  Connected: {gimbal['connected']}")
        if gimbal["connected"]:
            logger.info(f"  Attitude: Yaw={gimbal['yaw']:.1f}°, Pitch={gimbal['pitch']:.1f}°, Roll={gimbal['roll']:.1f}°")
            logger.info(f"  Locked: {gimbal['locked']}")

        # Camera status
        if self.camera:
            stats = self.camera.get_stats()
            logger.info("\nCamera:")
            logger.info(f"  Stream: {stats['opened']}")
            if stats["opened"]:
                logger.info(f"  Resolution: {stats['resolution']}")
                logger.info(f"  FPS: {stats['fps']}")
                logger.info(f"  Total Frames: {stats['total_frames']}")

        # Detection status
        logger.info(f"\nDetections: {summary['detections']}")

        # Mission status
        mission = summary["mission"]
        logger.info("\nMission:")
        logger.info(f"  Mode: {mission['mode'].upper()}")
        logger.info(f"  Active: {mission['active']}")
        if mission["target"]:
            logger.info(f"  Target: {mission['target']}")

        # Errors
        if summary["errors"] > 0:
            logger.info(f"\nErrors: {summary['errors']}")

        logger.info("-" * 60)

    def get_state(self) -> SystemState:
        """
        Get reference to global state.

        Returns:
            SystemState instance
        """
        return self.state

    def get_config(self) -> Dict[str, Any]:
        """
        Get system configuration.

        Returns:
            Configuration dictionary
        """
        return self.config

    # Tracking control methods (Phase 4)

    def start_tracking(self, track_id: int) -> bool:
        """
        Start autonomous tracking of a specific target.

        Args:
            track_id: Track ID to follow

        Returns:
            True if tracking started successfully
        """
        if not self.tracker:
            logger.error("Tracker not initialized")
            return False

        if self.tracker.lock_target(track_id):
            self._tracking_enabled = True
            logger.info("Started autonomous tracking: Track ID %s", track_id)
            return True

        return False

    def stop_tracking(self):
        """Stop autonomous tracking."""
        if self.tracker:
            self.tracker.unlock_target()
            self._tracking_enabled = False
            logger.info("Stopped autonomous tracking")

    def get_tracking_status(self) -> dict:
        """
        Get current tracking status with geolocation.

        Returns:
            Dictionary with tracking state, target info, and calculated location
        """
        if not self.tracker:
            return {"enabled": False, "state": "NOT_INITIALIZED", "target": None}

        target = self.tracker.get_target()
        if not target:
            return {
                "enabled": self._tracking_enabled,
                "state": self.tracker.get_state().value,
                "target": None,
            }

        # Calculate target geolocation if geolocator available
        target_location = None
        if self.geolocator:
            drone_state = self.state.get_drone_state()
            gimbal_state = self.state.get_gimbal_state()

            if (
                drone_state.latitude
                and drone_state.longitude
                and drone_state.altitude_relative > 0
                and gimbal_state.pitch is not None
            ):

                try:
                    location = self.geolocator.pixel_to_gps(
                        pixel_x=target.center_x,
                        pixel_y=target.center_y,
                        drone_lat=drone_state.latitude,
                        drone_lon=drone_state.longitude,
                        drone_alt=drone_state.altitude_relative,
                        drone_roll=drone_state.roll or 0.0,
                        drone_pitch=drone_state.pitch or 0.0,
                        drone_yaw=drone_state.yaw or 0.0,
                        gimbal_pitch=gimbal_state.pitch,
                        gimbal_yaw=gimbal_state.yaw or 0.0,
                    )

                    if location:
                        # Calculate vertical bearing angle for slant range
                        y_norm = (target.center_y - self.geolocator.intrinsics.cy) / \
                                 self.geolocator.intrinsics.focal_length_px
                        bearing_y = math.atan(y_norm)

                        slant_range = self.geolocator.calculate_slant_range(
                            drone_alt=drone_state.altitude_relative,
                            drone_pitch=drone_state.pitch or 0.0,
                            gimbal_pitch=gimbal_state.pitch,
                            bearing_y=bearing_y,
                        )

                        target_location = {
                            "latitude": location[0],
                            "longitude": location[1],
                            "distance": slant_range,
                        }
                except Exception as e:
                    logger.warning("Geolocation calculation failed: %s", e)

        return {
            "enabled": self._tracking_enabled,
            "state": self.tracker.get_state().value,
            "target": {
                "track_id": target.track_id,
                "class_name": target.class_name,
                "confidence": target.confidence,
                "frames_tracked": target.frames_tracked,
                "frames_lost": target.frames_lost,
                "location": target_location,
            },
        }
