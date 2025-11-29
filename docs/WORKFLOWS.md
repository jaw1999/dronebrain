# System Workflows

Common workflows and operational procedures for DroneBrain.

## System Startup

### Standard Startup

```bash
# 1. Connect hardware
# - Ethernet to flat LAN
# - Power on drone
# - Power on camera/gimbal

# 2. Verify network connectivity
ping 192.168.144.102  # Drone
ping 192.168.144.25   # Camera

# 3. Start DroneBrain
cd /home/jordan/development/dronebrain
./start.sh

# 4. Verify services
# - MediaMTX started
# - MAVLink connected
# - Siyi connected
# - RTSP capture active
# - YOLOv8 loaded
# - Web server on port 8080

# 5. Access web interface
# Open browser: http://localhost:8080
```

### Development Startup

```bash
# Start with debug logging
./start.sh --dev

# Watch logs
tail -f logs/dronebrain.log
```

## Target Detection Workflow

### 1. Video Feed Setup

```
User → Web UI → WebRTC Player
                    ↓
              MediaMTX Server
                    ↓
              FFmpeg Publisher
                    ↓
          Annotated Video Stream
                    ↓
            Vision Pipeline
                    ↓
         YOLOv8 Detector (GPU)
```

**Process:**
1. RTSP capture reads from Siyi camera
2. YOLOv8 runs detection on each frame (GPU)
3. Bounding boxes annotated on frame
4. FFmpeg publishes to MediaMTX
5. Web UI displays via WebRTC

### 2. Object Detection

```
Frame → YOLOv8 → Detections → Tracker → Track IDs
```

**Parameters:**
- Confidence threshold: 0.25
- IOU threshold: 0.45
- Classes: [0,1,2,3,5,7] (person, bicycle, car, motorcycle, bus, truck)

**Output:**
- Bounding box (x1, y1, x2, y2)
- Class name
- Confidence score
- Track ID (persistent across frames)

## Target Tracking Workflow

### 1. Lock Target

```
User → Enter Track ID → Start Tracking
                            ↓
                    Target Tracker
                            ↓
                    PID Controller
                            ↓
                    Gimbal Commands
                            ↓
                    Siyi Interface
                            ↓
                    Gimbal Motors
```

**Process:**
1. User selects track ID from web UI
2. System locks onto track
3. PID controller calculates gimbal commands
4. Gimbal tracks target in frame center

**PID Loop (20Hz):**
```python
# Calculate error
error_x = target_x - frame_center_x
error_y = target_y - frame_center_y

# PID calculation
output_x = Kp * error_x + Ki * integral_x + Kd * derivative_x
output_y = Kp * error_y + Ki * integral_y + Kd * derivative_y

# Apply limits
pitch_cmd = clamp(current_pitch + output_y, -90, 25)
yaw_cmd = clamp(current_yaw + output_x, -135, 135)

# Send to gimbal
gimbal.set_angles(pitch_cmd, yaw_cmd, speed)
```

### 2. Geolocation

```
Target Pixel Coords → Geolocator → GPS Coords
        +                             ↓
  Drone Telemetry              Target Position
        +                             ↓
  Gimbal Angles                  TAK Server
        +                             ↓
  Camera Intrinsics            ATAK Clients
```

**Transform Chain:**
1. Pixel → Camera ray
2. Camera → Gimbal (rotate by gimbal angles)
3. Gimbal → Body (mounting transform)
4. Body → NED (rotate by drone attitude)
5. NED → Ground intersection (ray-plane)
6. NED → GPS (add to drone position)

**Update Rate:**
- Geolocation: 10Hz
- TAK publish: 1Hz

## Autonomous Flight Workflow

### Loiter Mode

```
User → Select Loiter → Verify Prerequisites → Enable Mode
                                ↓
                        Autonomous Manager
                                ↓
                     Waypoint Generation
                                ↓
                        MAVLink Commands
                                ↓
                          Autopilot
                                ↓
                       Circular Orbit
```

**Steps:**
1. Lock target
2. Verify armed + GUIDED mode
3. Click "Loiter Mode"
4. System generates orbit waypoints
5. Drone flies circular path
6. Gimbal tracks target

**Orbit Calculation:**
```python
# Every 2 seconds
current_bearing = bearing(drone, target)
angle_increment = (loiter_speed / standoff_distance) * update_interval
next_bearing = (current_bearing + angle_increment) % 360
next_point = destination(target, next_bearing, standoff_distance)
mavlink.goto(next_point, orbit_altitude)
```

### Follow Mode

```
User → Select Follow → Verify Prerequisites → Enable Mode
                                ↓
                        Autonomous Manager
                                ↓
                     Track Target Movement
                                ↓
                    Calculate Follow Position
                                ↓
                        MAVLink Commands
                                ↓
                          Autopilot
                                ↓
                        Chase Target
```

**Steps:**
1. Lock moving target
2. Verify armed + GUIDED mode
3. Click "Follow Mode"
4. System tracks target GPS
5. Calculates follow position (standoff distance behind)
6. Updates waypoint when target moves >5m

**Follow Calculation:**
```python
# Every 2 seconds
if target_moved > min_movement:
    bearing_to_target = bearing(drone, target)
    follow_bearing = (bearing_to_target + 180) % 360
    follow_point = destination(target, follow_bearing, standoff_distance)
    mavlink.goto(follow_point, current_altitude)
```

## Search Mode Workflow

### Automated Search Pattern

```
User → Click "Start Search" → Search Mode Active
                                    ↓
                            Gimbal Controller
                                    ↓
                         270° Sweep Pattern
                                    ↓
                          Object Detection
                                    ↓
                         Track List Updates
```

**Pattern:**
1. Start at -135° yaw
2. Sweep to +135° yaw at constant speed
3. Reverse direction
4. Repeat oscillation

**Parameters:**
- Pitch: -30° (looking down)
- Yaw range: -135° to +135° (270° total)
- Speed: 10-20 (adjustable)
- Detection runs continuously

## Mission Planning Workflow

### Pre-Flight

```
1. Hardware Check
   - Drone battery >80%
   - Camera/gimbal powered
   - Network connected
   - GPS fix >10 satellites

2. Software Check
   - DroneBrain running
   - MAVLink connected
   - Video streaming
   - Web UI accessible

3. Configuration
   - Detection classes enabled
   - Tracking parameters tuned
   - Autonomous settings configured
   - TAK server connected (if used)

4. Ground Test
   - Arm/disarm cycle
   - Gimbal movement
   - Mode changes
   - Video feed quality
```

### In-Flight

```
1. Takeoff
   - Manual takeoff to 10m
   - Verify telemetry
   - Check video feed

2. Detection Test
   - Point camera at ground
   - Verify detections appear
   - Check track IDs

3. Tracking Test
   - Lock onto stationary object
   - Verify gimbal tracks
   - Check GPS coordinates

4. Autonomous Test (Optional)
   - Enable loiter at low altitude
   - Verify orbit behavior
   - Monitor safety systems

5. Mission Execution
   - Increase altitude
   - Search for targets
   - Lock and track
   - Enable autonomous mode
   - Monitor battery

6. Recovery
   - Disable autonomous
   - Return to pilot control
   - RTL or manual landing
```

### Post-Flight

```
1. Data Review
   - Check logs for errors
   - Review detection accuracy
   - Analyze tracking performance
   - Verify geolocation accuracy

2. Video Archive
   - Save recorded video
   - Extract key frames
   - Generate reports

3. System Maintenance
   - Clear old logs
   - Update configuration
   - Calibrate sensors (if needed)
   - Recharge batteries
```

## Gimbal Control Workflow

### Manual Control

```
User → Web UI → REST API → MAVLink → Gimbal
```

**Controls:**
- D-pad: Pan/tilt
- Speed slider: Movement speed
- Center button: Reset to 0°/0°
- Zoom slider: Optical zoom

**Commands:**
```http
POST /api/gimbal/control
{
  "pitch": -45.0,
  "yaw": 15.0,
  "speed": 20
}
```

### Auto Tracking

```
Detection → Target Tracker → PID Loop → Gimbal
```

**PID Parameters:**
- P: 0.5 (proportional gain)
- I: 0.01 (integral gain)
- D: 0.1 (derivative gain)
- Max speed: 50
- Min speed: 5

**Tuning:**
- Increase P for faster response
- Increase D for damping
- Increase I for steady-state accuracy

## Telemetry Workflow

### Data Flow

```
Hardware → Interfaces → State Manager → Web Server → Clients
```

**Sources:**
- MAVLink: Drone telemetry (10Hz)
- Siyi: Gimbal state (5Hz)
- Vision: Detections (30Hz)
- Geolocator: Target GPS (10Hz)

**Distribution:**
- REST API: On-demand
- WebSocket: Streaming (10Hz)

**Message Format:**
```json
{
  "type": "telemetry",
  "timestamp": 1234567890.5,
  "drone": {...},
  "gimbal": {...},
  "tracking": {...},
  "autonomous": {...},
  "detections": [...]
}
```

## Error Recovery Workflow

### MAVLink Disconnection

```
1. Detect: Heartbeat timeout (5s)
2. Alert: Log error, UI notification
3. Disable: Stop autonomous modes
4. Reconnect: Retry connection every 5s
5. Resume: Restore state when reconnected
```

### Camera Loss

```
1. Detect: RTSP stream timeout
2. Alert: Log error, UI notification
3. Stop: Disable tracking and autonomous
4. Reconnect: Restart RTSP capture
5. Resume: Restart vision pipeline
```

### GPS Degradation

```
1. Detect: Satellites <6 or HDOP >2.0
2. Alert: UI warning
3. Restrict: Disable autonomous modes
4. Continue: Tracking and manual control OK
5. Recover: Re-enable when GPS improves
```

### Target Loss

```
1. Detect: Target not in detections for 5s
2. Hold: Continue last gimbal position
3. Search: Optionally enable search mode
4. Timeout: Disable tracking after 10s
5. Manual: Return gimbal control to user
```

### Low Battery

```
1. Warning (30%): UI notification
2. Critical (20%): Disable autonomous, suggest RTL
3. Emergency (15%): Force RTL
4. Failsafe (10%): Autopilot lands immediately
```

## Calibration Workflow

### Camera Calibration

```
1. Collect calibration images
   - Checkerboard pattern
   - Various angles and distances
   - 20-30 images

2. Run calibration
   python scripts/calibrate_camera.py

3. Update config
   focal_length_mm: <result>
   sensor_width_mm: <result>
   sensor_height_mm: <result>

4. Validate
   - Test geolocation accuracy
   - Compare with GPS ground truth
   - Adjust if needed
```

### Gimbal Calibration

```
1. Center gimbal
   POST /api/gimbal/center

2. Verify angles
   - Check pitch at 0°, -45°, -90°
   - Check yaw at 0°, ±90°, ±135°

3. Adjust offsets
   config:
     gimbal_pitch_offset: 0.0
     gimbal_yaw_offset: 0.0

4. Test tracking
   - Lock onto target
   - Verify smooth tracking
   - Tune PID if needed
```

### Compass Calibration

```
1. Use GCS (QGroundControl/Mission Planner)
2. Follow autopilot calibration procedure
3. Verify heading accuracy
4. Check yaw in DroneBrain telemetry
```

## Integration Workflow

### TAK Integration

```
1. Configure TAK server
   config.yaml:
     tak:
       enabled: true
       server_address: "10.x.x.x:8087"
       protocol: "udp"

2. Start DroneBrain
   ./start.sh

3. Verify connection
   # Check logs
   tail -f logs/dronebrain.log | grep TAK

4. Lock target
   # Should see CoT messages in logs

5. View in ATAK
   # Connect ATAK to TAK server
   # See targets on map
```

### VPN Access

```
1. Setup VPN server
   # OpenVPN, WireGuard, etc.

2. Configure Jetson as VPN client
   # Auto-connect on boot

3. Update firewall
   # Allow port 8080, 8554, 8889

4. Access from remote
   http://<vpn-ip>:8080
```

## Backup and Recovery

### Configuration Backup

```bash
# Backup
tar -czf dronebrain-config-$(date +%Y%m%d).tar.gz config/

# Restore
tar -xzf dronebrain-config-20250129.tar.gz
```

### Log Archiving

```bash
# Archive logs
tar -czf logs-$(date +%Y%m%d).tar.gz logs/
mv logs-*.tar.gz /backup/

# Clear old logs
rm logs/*.log
```

### System Image

```bash
# Create system image (Jetson)
sudo dd if=/dev/mmcblk0 of=/backup/jetson-$(date +%Y%m%d).img bs=4M

# Restore
sudo dd if=/backup/jetson-20250129.img of=/dev/mmcblk0 bs=4M
```
