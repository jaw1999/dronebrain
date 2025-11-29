"""Web UI Server.

FastAPI-based web interface for DroneBrain system.
Provides REST API, WebSocket telemetry, and serves the web UI.
"""

import logging
import asyncio
from typing import Optional, Dict, Any, List
from pathlib import Path
import threading

import yaml
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

logger = logging.getLogger(__name__)


class WebServer:
    """
    Web UI server for DroneBrain.

    Provides:
    - REST API for control and queries
    - WebSocket for real-time telemetry
    - Static file serving for web UI
    - Video stream proxying

    Example:
        >>> from src.core.pipeline_manager import PipelineManager
        >>> pipeline = PipelineManager()
        >>> web_server = WebServer(pipeline=pipeline, host="0.0.0.0", port=8080)
        >>> web_server.start()
    """

    def __init__(
        self,
        pipeline,
        host: str = "0.0.0.0",
        port: int = 8080,
    ):
        """
        Initialize web server.

        Args:
            pipeline: PipelineManager instance
            host: Host address to bind to
            port: Port to listen on
        """
        self.pipeline = pipeline
        self.host = host
        self.port = port

        # FastAPI app
        self.app = FastAPI(
            title="DroneBrain Web UI",
            description="Web interface for DroneBrain autonomous drone system",
            version="1.0.0",
        )

        # WebSocket connections
        self.websocket_clients: List[WebSocket] = []

        # Server thread
        self._server_thread: Optional[threading.Thread] = None
        self._running = False

        # Configure CORS - permissive for development/field use
        # Security note: Protected by input validation
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  # Allow all origins for remote field access
            allow_credentials=True,
            allow_methods=["GET", "POST", "PUT", "DELETE"],
            allow_headers=["*"],
        )

        # Setup routes
        self._setup_routes()

    def _setup_routes(self):
        """Setup API routes and endpoints."""

        # Serve static files (web UI)
        static_dir = Path(__file__).parent / "static"
        if static_dir.exists():
            self.app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

        # Main page - serve v2 by default
        @self.app.get("/", response_class=HTMLResponse)
        async def root():
            """Serve main web UI page."""
            # Try v2 first, fall back to v1
            index_file_v2 = Path(__file__).parent / "static" / "index_v2.html"
            index_file = Path(__file__).parent / "static" / "index.html"

            if index_file_v2.exists():
                return FileResponse(index_file_v2)
            elif index_file.exists():
                return FileResponse(index_file)
            else:
                return HTMLResponse(
                    "<h1>DroneBrain Web UI</h1><p>Static files not found. "
                    "Please build the web UI first.</p>"
                )

        # Health check
        @self.app.get("/api/health")
        async def health():
            """Health check endpoint."""
            return {"status": "ok", "service": "dronebrain-web-ui"}

        # System status
        @self.app.get("/api/status")
        async def get_status():
            """Get system status."""
            if not self.pipeline:
                raise HTTPException(status_code=503, detail="Pipeline not initialized")

            summary = self.pipeline.state.get_summary()
            return {
                "status": summary["status"].value,
                "drone": summary["drone"],
                "gimbal": summary["gimbal"],
                "camera": summary["camera"],
                "detections": summary.get("detections", 0),
            }

        # Drone telemetry
        @self.app.get("/api/telemetry/drone")
        async def get_drone_telemetry():
            """Get drone telemetry data."""
            if not self.pipeline:
                raise HTTPException(status_code=503, detail="Pipeline not initialized")

            drone_state = self.pipeline.state.get_drone_state()
            return {
                "connected": drone_state.connected,
                "armed": drone_state.armed,
                "mode": drone_state.mode,
                "position": {
                    "latitude": drone_state.latitude,
                    "longitude": drone_state.longitude,
                    "altitude": drone_state.altitude,
                    "altitude_relative": drone_state.altitude_relative,
                },
                "attitude": {
                    "roll": drone_state.roll,
                    "pitch": drone_state.pitch,
                    "yaw": drone_state.yaw,
                },
                "velocity": {
                    "vx": drone_state.vx,
                    "vy": drone_state.vy,
                    "vz": drone_state.vz,
                },
                "battery": drone_state.battery_percent,
                "gps_fix": drone_state.gps_fix_type,
                "satellites": drone_state.satellites_visible,
            }

        # Gimbal telemetry
        @self.app.get("/api/telemetry/gimbal")
        async def get_gimbal_telemetry():
            """Get gimbal telemetry data."""
            if not self.pipeline:
                raise HTTPException(status_code=503, detail="Pipeline not initialized")

            gimbal_state = self.pipeline.state.get_gimbal_state()
            return {
                "connected": gimbal_state.connected,
                "yaw": gimbal_state.yaw,
                "pitch": gimbal_state.pitch,
                "roll": gimbal_state.roll,
                "locked": gimbal_state.locked,
            }

        # Tracking status
        @self.app.get("/api/tracking/status")
        async def get_tracking_status():
            """Get tracking status."""
            if not self.pipeline or not self.pipeline.tracker:
                return {"enabled": False, "state": "IDLE", "target": None}

            return self.pipeline.get_tracking_status()

        # Start tracking
        @self.app.post("/api/tracking/start/{track_id}")
        async def start_tracking(track_id: int):
            """Start tracking a target by track ID."""
            if not self.pipeline or not self.pipeline.tracker:
                raise HTTPException(status_code=503, detail="Tracker not available")

            success = self.pipeline.start_tracking(track_id)
            return {"success": success, "track_id": track_id}

        # Stop tracking
        @self.app.post("/api/tracking/stop")
        async def stop_tracking():
            """Stop tracking current target."""
            if not self.pipeline or not self.pipeline.tracker:
                raise HTTPException(status_code=503, detail="Tracker not available")

            self.pipeline.stop_tracking()
            return {"success": True}

        # Autonomous mode status
        @self.app.get("/api/autonomous/status")
        async def get_autonomous_status():
            """Get autonomous mode status."""
            if not self.pipeline or not self.pipeline.autonomous_manager:
                raise HTTPException(status_code=503, detail="Autonomous manager not available")

            return self.pipeline.autonomous_manager.get_status()

        # Enable loiter mode
        @self.app.post("/api/autonomous/loiter")
        async def enable_loiter_mode():
            """Enable loiter (orbit) mode around locked target."""
            if not self.pipeline or not self.pipeline.autonomous_manager:
                raise HTTPException(status_code=503, detail="Autonomous manager not available")

            from ..autonomous.autonomous_manager import AutonomousMode
            success = self.pipeline.autonomous_manager.set_mode(AutonomousMode.LOITER)

            if not success:
                raise HTTPException(
                    status_code=400,
                    detail="Failed to enable loiter mode. Check: drone armed, GUIDED mode, target locked"
                )

            return {"success": True, "mode": "loiter"}

        # Enable follow mode
        @self.app.post("/api/autonomous/follow")
        async def enable_follow_mode():
            """Enable follow mode to chase moving target."""
            if not self.pipeline or not self.pipeline.autonomous_manager:
                raise HTTPException(status_code=503, detail="Autonomous manager not available")

            from ..autonomous.autonomous_manager import AutonomousMode
            success = self.pipeline.autonomous_manager.set_mode(AutonomousMode.FOLLOW)

            if not success:
                raise HTTPException(
                    status_code=400,
                    detail="Failed to enable follow mode. Check: drone armed, GUIDED mode, target locked"
                )

            return {"success": True, "mode": "follow"}

        # Disable autonomous mode
        @self.app.post("/api/autonomous/disable")
        async def disable_autonomous_mode():
            """Disable autonomous flight mode."""
            if not self.pipeline or not self.pipeline.autonomous_manager:
                raise HTTPException(status_code=503, detail="Autonomous manager not available")

            from ..autonomous.autonomous_manager import AutonomousMode
            success = self.pipeline.autonomous_manager.set_mode(AutonomousMode.DISABLED)
            return {"success": success, "mode": "disabled"}

        # Update standoff distance
        @self.app.post("/api/autonomous/standoff")
        async def set_standoff_distance(distance: float):
            """Set standoff distance from target (meters)."""
            if not self.pipeline or not self.pipeline.autonomous_manager:
                raise HTTPException(status_code=503, detail="Autonomous manager not available")

            # Input validation - reasonable range for drone operations
            distance = max(10.0, min(500.0, distance))

            self.pipeline.autonomous_manager.standoff_distance = distance

            # Also update config so it persists
            if "autonomous" not in self.pipeline.config:
                self.pipeline.config["autonomous"] = {}
            self.pipeline.config["autonomous"]["standoff_distance"] = distance

            return {
                "success": True,
                "standoff_distance": distance
            }

        # Gimbal control - center
        @self.app.post("/api/gimbal/center")
        async def center_gimbal():
            """Center the gimbal."""
            if not self.pipeline or not self.pipeline.siyi:
                raise HTTPException(status_code=503, detail="Gimbal not available")

            success = self.pipeline.siyi.center()
            return {"success": success}

        # Gimbal control - set attitude
        @self.app.post("/api/gimbal/attitude")
        async def set_gimbal_attitude(yaw: Optional[float] = None, pitch: Optional[float] = None):
            """Set gimbal attitude."""
            if not self.pipeline or not self.pipeline.siyi:
                raise HTTPException(status_code=503, detail="Gimbal not available")

            # Input validation - clamp to safe ranges
            if yaw is not None:
                yaw = max(-135.0, min(135.0, yaw))
            if pitch is not None:
                pitch = max(-90.0, min(25.0, pitch))

            success = self.pipeline.siyi.set_attitude(yaw=yaw, pitch=pitch)
            return {"success": success, "yaw": yaw, "pitch": pitch}

        # Gimbal control - manual rotation
        @self.app.post("/api/gimbal/move")
        async def move_gimbal(yaw_speed: int = 0, pitch_speed: int = 0):
            """Move gimbal at specified speed."""
            if not self.pipeline or not self.pipeline.siyi:
                raise HTTPException(status_code=503, detail="Gimbal not available")

            # Input validation - clamp to safe speed ranges
            yaw_speed = max(-100, min(100, yaw_speed))
            pitch_speed = max(-100, min(100, pitch_speed))

            success = self.pipeline.siyi.set_gimbal_rotation(yaw_speed=yaw_speed, pitch_speed=pitch_speed)
            return {"success": success, "yaw_speed": yaw_speed, "pitch_speed": pitch_speed}

        # Zoom control
        @self.app.post("/api/gimbal/zoom")
        async def control_zoom(level: int):
            """Control zoom (1=in, -1=out, 0=stop)."""
            if not self.pipeline or not self.pipeline.siyi:
                raise HTTPException(status_code=503, detail="Gimbal not available")

            # Input validation - only allow -1, 0, 1
            level = max(-1, min(1, level))

            success = self.pipeline.siyi.zoom(level=level)
            return {"success": success, "level": level}

        # Configuration management
        @self.app.get("/api/config")
        async def get_config():
            """Get current configuration."""
            config_path = Path(__file__).parent.parent.parent / "config" / "config.yaml"

            if not config_path.exists():
                raise HTTPException(status_code=404, detail="Configuration file not found")

            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            return config

        @self.app.post("/api/config")
        async def update_config(config: Dict[str, Any]):
            """Update configuration."""
            config_path = Path(__file__).parent.parent.parent / "config" / "config.yaml"

            try:
                # Basic validation - ensure required sections exist
                required_sections = ["network", "drone", "camera", "vision", "web", "logging"]
                for section in required_sections:
                    if section not in config:
                        raise HTTPException(
                            status_code=400,
                            detail=f"Invalid configuration: missing required section '{section}'"
                        )

                # Validate gimbal speed limits in tracking config
                if "vision" in config and "tracking" in config["vision"]:
                    tracking = config["vision"]["tracking"]
                    if "max_gimbal_speed" in tracking:
                        # Clamp to safe range
                        tracking["max_gimbal_speed"] = max(1, min(200, tracking["max_gimbal_speed"]))

                with open(config_path, "w") as f:
                    yaml.dump(config, f, default_flow_style=False, sort_keys=False)

                logger.info("Configuration updated via web UI")
                return {
                    "success": True,
                    "message": "Configuration updated. Restart required for changes to take effect.",
                }
            except HTTPException:
                raise
            except Exception as e:
                logger.error(f"Failed to update configuration: {e}")
                raise HTTPException(
                    status_code=500, detail=f"Failed to update configuration: {str(e)}"
                )

        # WebSocket for real-time telemetry
        @self.app.websocket("/ws/telemetry")
        async def websocket_telemetry(websocket: WebSocket):
            """WebSocket endpoint for real-time telemetry streaming."""
            await websocket.accept()
            self.websocket_clients.append(websocket)
            logger.info(
                f"WebSocket client connected. " f"Total clients: {len(self.websocket_clients)}"
            )

            try:
                # Keep connection alive and send telemetry
                while True:
                    # Send telemetry data
                    if self.pipeline:
                        summary = self.pipeline.state.get_summary()
                        tracking_status = (
                            self.pipeline.get_tracking_status()
                            if self.pipeline.tracker
                            else {"enabled": False}
                        )

                        # Get TAK stats if available
                        tak_stats = {"enabled": False, "connected": False, "messages_sent": 0}
                        if hasattr(self.pipeline, "tak_client") and self.pipeline.tak_client:
                            try:
                                stats = self.pipeline.tak_client.get_stats()
                                tak_stats = {
                                    "enabled": True,
                                    "connected": stats.get("connected", False),
                                    "messages_sent": stats.get("messages_sent", 0),
                                    "messages_failed": stats.get("messages_failed", 0),
                                }
                            except Exception as e:
                                logger.warning(f"Failed to get TAK stats: {e}")

                        # Get autonomous mode status if available
                        autonomous_status = {"available": False, "enabled": False, "mode": "disabled"}
                        if hasattr(self.pipeline, "autonomous_manager") and self.pipeline.autonomous_manager:
                            try:
                                autonomous_status = self.pipeline.autonomous_manager.get_status()
                                autonomous_status["available"] = True
                            except Exception as e:
                                logger.warning(f"Failed to get autonomous status: {e}")

                        # Get active tracks with GPS coordinates
                        # Only include detections that have track IDs assigned
                        tracks = []
                        detections = self.pipeline.state.get_all_detections()
                        for detection in detections:
                            # Skip detections without track IDs
                            if detection.track_id is None:
                                continue

                            track_data = {
                                "track_id": detection.track_id,
                                "class_name": detection.class_name,
                                "confidence": detection.confidence,
                                "bbox": detection.bbox,
                            }
                            # Add GPS data if available
                            if detection.latitude is not None:
                                track_data["latitude"] = detection.latitude
                            if detection.longitude is not None:
                                track_data["longitude"] = detection.longitude
                            if detection.distance is not None:
                                track_data["distance"] = detection.distance

                            tracks.append(track_data)

                        telemetry_data = {
                            "type": "telemetry",
                            "timestamp": summary.get("timestamp", 0),
                            "status": summary["status"].value,
                            "drone": summary["drone"],
                            "gimbal": summary["gimbal"],
                            "camera": summary["camera"],
                            "detections": summary.get("detections", 0),
                            "tracks": tracks,
                            "tracking": tracking_status,
                            "tak": tak_stats,
                            "autonomous": autonomous_status,
                        }

                        await websocket.send_json(telemetry_data)

                    # Send at 5Hz
                    await asyncio.sleep(0.2)

            except WebSocketDisconnect:
                logger.info("WebSocket client disconnected")
            except Exception as e:
                logger.error(f"WebSocket error: {e}")
            finally:
                if websocket in self.websocket_clients:
                    self.websocket_clients.remove(websocket)
                logger.info(
                    f"WebSocket client removed. " f"Total clients: {len(self.websocket_clients)}"
                )

    def start(self):
        """Start web server in background thread."""
        if self._running:
            logger.warning("Web server already running")
            return

        self._running = True

        # Run uvicorn server in background thread
        def run_server():
            config = uvicorn.Config(
                self.app,
                host=self.host,
                port=self.port,
                log_level="info",
                access_log=False,
            )
            server = uvicorn.Server(config)
            server.run()

        self._server_thread = threading.Thread(target=run_server, daemon=True)
        self._server_thread.start()

        logger.info(f"Web UI started at http://{self.host}:{self.port}")

    def stop(self):
        """Stop web server."""
        if not self._running:
            return

        self._running = False

        # Close all WebSocket connections
        # Note: WebSocket cleanup happens automatically when connections are closed
        # or when the server shuts down. Manual cleanup can cause event loop issues.
        self.websocket_clients.clear()

        logger.info("Web UI stopped")

    def is_running(self) -> bool:
        """Check if web server is running."""
        return self._running
