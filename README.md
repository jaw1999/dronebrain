# DroneBrain - Autonomous Drone Vision & Control System

## Overview

DroneBrain is an intelligent autonomous drone system that combines real-time computer vision, gimbal stabilization, and flight control for reconnaissance and target tracking missions. The system runs on NVIDIA Jetson and integrates:

- **CubeOrange Flight Controller** (MAVLink protocol)
- **Siyi A8 Mini Gimbal Camera** (Custom UDP protocol + RTSP streaming)
- **YOLOv8 Vision Model** (GPU-accelerated object detection and tracking)
- **MediaMTX Streaming Server** (Multi-protocol video distribution)
- **TAK Server Integration** (Cursor-on-Target messaging)
- **Web-based Control Interface** (Real-time telemetry and control)

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      NVIDIA Jetson (DroneBrain)             │
│                                                             │
│  Core Pipeline → Vision Processing → Target Tracking        │
│       ↓                   ↓                    ↓            │
│  MAVLink/Siyi         YOLOv8            Photogrammetry      │
│       ↓                   ↓                    ↓            │
│  Autonomous    →    MediaMTX Stream   →   CoT/TAK           │
│   Behaviors              Web UI              Server         │
└─────────────────────────────────────────────────────────────┘
```

## Features

### Vision & Detection
- Real-time object detection using YOLOv8 with tracking
- Multi-object tracking with persistent track IDs
- Configurable detection classes: person, vehicle, boat, aircraft
- GPU-accelerated inference for low latency
- Bounding box and track ID annotation on video stream

### Target Tracking & Geolocation
- YOLOv8 built-in tracker with persistent IDs
- Photogrammetry-based GPS coordinate calculation
- Accounts for drone position, attitude, and gimbal angles
- Ground-level target assumption
- Real-time position updates for locked targets

### Autonomous Behaviors
- **PID Gimbal Tracking**: Locked target tracking with PID control
- **Search Mode**: 270° oscillating sweep pattern for target acquisition
- **Loiter Mode**: Circular orbit around locked target at standoff distance
- **Follow Mode**: Chase moving targets maintaining standoff distance
- Automatic waypoint generation via MAVLink GUIDED mode
- Safety checks: armed, GUIDED mode, target lock required

### Streaming & Distribution
- Annotated video stream with detection overlays
- MediaMTX multi-protocol support (RTSP/RTMP/HLS/WebRTC)
- FFmpeg hardware acceleration
- Configurable quality and framerate

### TAK Integration
- Cursor-on-Target (CoT) messaging protocol
- Real-time target position updates to TAK server
- Configurable update intervals
- Target classification (friendly/neutral/hostile)
- Entity types: person/vehicle/boat/aircraft

### Web Interface
- WebSocket telemetry (10Hz update rate)
- WebRTC video feed integration
- Manual gimbal control (pan/tilt/zoom)
- Track selection and locking
- Autonomous mode controls (loiter/follow)
- Standoff distance configuration
- System status monitoring

## Hardware Requirements

### Compute Platform
- NVIDIA Jetson (Orin Nano/Xavier NX/AGX recommended)
- 8GB+ RAM
- 32GB+ storage
- CUDA-capable GPU

### Network
- Ethernet connectivity to flat LAN (192.168.144.x/24)
- VPN capability for remote access
- Starlink or stable internet connection

### Connected Devices
- CubeOrange flight controller (MAVLink over UDP)
- Siyi A8 Mini gimbal camera (Ethernet)
- MediaMTX server (local or remote)
- TAK server (over VPN)

## Software Requirements

### System
- Ubuntu 20.04/22.04 (JetPack for Jetson)
- Python 3.10+
- CUDA 11.x or 12.x
- FFmpeg with hardware acceleration

### Python Dependencies
See `requirements.txt` for complete list:
- PyTorch (with CUDA support)
- ultralytics (YOLOv8)
- OpenCV (cv2)
- PyMAVLink
- FastAPI + uvicorn
- websockets
- pytak (CoT messaging)
- pyyaml

## Installation

### 1. Clone Repository
```bash
git clone https://github.com/jaw1999/dronebrain.git
cd dronebrain
```

### 2. Install System Dependencies
```bash
# FFmpeg with hardware acceleration
sudo apt update
sudo apt install ffmpeg libavcodec-dev libavformat-dev libavutil-dev

# MediaMTX (optional, can run on separate server)
wget https://github.com/bluenviron/mediamtx/releases/download/v1.5.0/mediamtx_v1.5.0_linux_arm64v8.tar.gz
tar -xzf mediamtx_v1.5.0_linux_arm64v8.tar.gz
sudo mv mediamtx /usr/local/bin/
```

### 3. Install Python Dependencies
```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install requirements
pip install -r requirements.txt
```

### 4. Download Vision Models
```bash
# YOLOv8 will auto-download on first run
# Or manually download:
# yolo task=detect mode=predict model=yolov8n.pt
```

### 5. Configure System
```bash
# Copy example config
cp config/config.example.yaml config/config.yaml

# Edit configuration
nano config/config.yaml
```

### 6. Configure MediaMTX
```bash
# MediaMTX config already included
# Edit if needed: config/mediamtx.yml
```

## Configuration

### Network Setup
All devices on same subnet:
- Jetson: `192.168.144.x`
- CubeOrange: `192.168.144.102` (MAVLink UDP)
- Siyi A8 Mini: `192.168.144.25` (UDP + RTSP)

### config.yaml Structure
```yaml
drone:
  connection_string: "udp:192.168.144.102:14550"
  heartbeat_timeout: 5

camera:
  siyi_ip: "192.168.144.25"
  siyi_port: 37260
  rtsp_url: "rtsp://192.168.144.25:8554/main.264"
  focal_length_mm: 21.0
  sensor_width_mm: 6.3
  sensor_height_mm: 4.7

vision:
  enabled: true
  model_path: "yolov8n.pt"
  confidence: 0.25
  iou: 0.45
  device: "cuda"
  classes: [0, 1, 2, 3, 5, 7]  # person, bicycle, car, motorcycle, bus, truck

tracker:
  enabled: true
  pid_p: 0.5
  pid_i: 0.01
  pid_d: 0.1
  max_speed: 50
  min_speed: 5

streaming:
  enabled: true
  rtsp_server: "rtsp://localhost:8554"
  stream_name: "drone"
  fps: 30
  bitrate: "2000k"

tak:
  enabled: true
  server_address: "10.x.x.x:8087"
  protocol: "udp"
  callsign: "DRONEBRAIN-1"

autonomous:
  standoff_distance: 50.0
  loiter_altitude_offset: 0.0
  loiter_speed: 5.0
  follow_speed: 8.0
  waypoint_update_interval: 2.0
  min_target_movement: 5.0

web:
  enabled: true
  host: "0.0.0.0"
  port: 8080
```

## Usage

### Start DroneBrain System
```bash
# Start system (automatically handles venv and MediaMTX)
./start.sh

# Development mode with debug logging
./start.sh --dev
```

### Stop DroneBrain System
```bash
# Graceful shutdown
./stop.sh
```

### Access Web Interface
Navigate to: `http://<jetson-ip>:8080`

Over VPN: `http://<vpn-ip>:8080`

### Basic Workflow

1. **System Startup**
   - start.sh launches MediaMTX and DroneBrain
   - MAVLink connects to CubeOrange (192.168.144.102:14550)
   - Siyi interface connects to camera (192.168.144.25:37260)
   - RTSP capture starts from camera stream
   - YOLOv8 loads to GPU
   - Web server starts on port 8080

2. **Video Streaming**
   - FFmpeg publishes annotated feed to MediaMTX
   - WebRTC available at http://localhost:8889
   - RTSP at rtsp://localhost:8554
   - HLS at http://localhost:8888

3. **Target Detection**
   - YOLOv8 runs on each frame
   - Detections annotated with bounding boxes and track IDs
   - Track list updates in web UI

4. **Target Locking**
   - Enter track ID in web UI
   - Click "Start" to lock target
   - PID controller drives gimbal to track target
   - Photogrammetry calculates GPS coordinates

5. **Autonomous Flight**
   - Requires: Armed + GUIDED mode + locked target
   - Set standoff distance (10-500m)
   - Select "Loiter" (circular orbit) or "Follow" (chase)
   - System sends waypoints via MAVLink

6. **TAK Integration**
   - TAK client publishes CoT messages
   - Locked target appears in TAK server
   - Update interval configurable in config.yaml

## Documentation

- [API Reference](docs/API.md) - REST and WebSocket API documentation
- [Photogrammetry](docs/PHOTOGRAMMETRY.md) - GPS coordinate calculation math
- [Autonomous Flight](docs/AUTONOMOUS_FLIGHT.md) - Loiter and Follow mode algorithms
- [Workflows](docs/WORKFLOWS.md) - Operational procedures and common workflows
- [TAK Integration](docs/TAK_INTEGRATION.md) - Cursor-on-Target messaging setup

## Project Structure

```
dronebrain/
├── src/
│   ├── core/
│   │   ├── pipeline_manager.py   # Main orchestration loop
│   │   └── state.py               # Global state (drone/gimbal/tracking)
│   ├── interfaces/
│   │   ├── mavlink_interface.py  # CubeOrange MAVLink
│   │   ├── siyi_interface.py     # Siyi gimbal UDP
│   │   └── rtsp_capture.py       # RTSP video capture
│   ├── vision/
│   │   └── yolo_detector.py      # YOLOv8 detection + tracking
│   ├── tracking/
│   │   └── geolocator.py         # Photogrammetry GPS calculation
│   ├── control/
│   │   ├── pid_controller.py     # PID gimbal tracking
│   │   └── target_tracker.py     # Target lock management
│   ├── autonomous/
│   │   └── autonomous_manager.py # Loiter/Follow modes
│   ├── streaming/
│   │   └── stream_publisher.py   # FFmpeg → MediaMTX
│   ├── interfaces/
│   │   ├── cot_generator.py      # CoT XML generation
│   │   └── tak_interface.py      # TAK server client
│   └── web/
│       ├── web_server.py         # FastAPI REST + WebSocket
│       └── static/
│           ├── index_v2.html
│           ├── css/style_v2.css
│           └── js/app_v2.js
├── config/
│   ├── config.yaml               # Main configuration
│   ├── config.example.yaml       # Example config
│   ├── drone_tracker.yaml        # Tracker PID tuning
│   └── mediamtx.yml              # MediaMTX config
├── docs/
│   ├── API.md                    # API documentation
│   ├── PHOTOGRAMMETRY.md         # Geolocation math
│   ├── AUTONOMOUS_FLIGHT.md      # Flight mode algorithms
│   ├── WORKFLOWS.md              # Operational procedures
│   └── TAK_INTEGRATION.md        # TAK setup guide
├── examples/
│   ├── tak_demo.py
│   ├── test_gimbal.py
│   └── tracking_demo.py
├── scripts/
│   └── cleanup_gpu.sh
├── start.sh                      # Startup script
├── stop.sh                       # Shutdown script
├── main.py                       # Entry point
├── requirements.txt
└── README.md
```

## Development

### Running Tests
```bash
pytest tests/
```

### Development Mode
```bash
# Run with auto-reload
python main.py --dev
```

### Logging
DroneBrain: `logs/dronebrain.log`
MediaMTX: `logs/mediamtx.log`

View logs:
```bash
tail -f logs/dronebrain.log
tail -f logs/mediamtx.log
```

## Troubleshooting

### Camera Connection Issues
```bash
# Ping camera
ping 192.168.144.25

# Test RTSP stream
ffplay rtsp://192.168.144.25:8554/main.264

# Test gimbal control
python examples/test_gimbal.py
```

### MAVLink Connection Issues
```bash
# Ping flight controller
ping 192.168.144.102

# Test MAVLink
mavproxy.py --master=udp:192.168.144.102:14550
```

### GPU/CUDA Issues
```bash
# Verify CUDA
python -c "import torch; print(torch.cuda.is_available())"

# Check GPU utilization
nvidia-smi
```

### MediaMTX Issues
```bash
# Check if running
pgrep mediamtx

# View logs
tail -f logs/mediamtx.log

# Restart
./stop.sh && ./start.sh
```

## Performance Optimization

### Jetson Power Mode
```bash
# Set to max performance
sudo nvpmodel -m 0
sudo jetson_clocks
```

### Vision Model Optimization
- YOLOv8n (nano) for speed, YOLOv8m (medium) for accuracy
- Adjust confidence threshold in config.yaml
- Reduce FPS if GPU overloaded
- Use INT8 quantization for TensorRT

### Network Optimization
- Wired Ethernet for camera and flight controller
- Adjust streaming bitrate in config.yaml
- Lower FPS over VPN connections

