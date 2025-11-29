# Autonomous Flight Modes

DroneBrain implements autonomous flight behaviors for target tracking and reconnaissance.

## Overview

Two autonomous modes:
- **Loiter**: Circular orbit around stationary target
- **Follow**: Chase moving target maintaining standoff distance

## Architecture

```
┌──────────────────┐
│ Vision Pipeline  │
│   (Detections)   │
└────────┬─────────┘
         │
         ↓
┌──────────────────┐
│ Target Tracker   │
│  (Lock Target)   │
└────────┬─────────┘
         │
         ↓
┌──────────────────┐
│   Geolocator     │
│  (GPS Coords)    │
└────────┬─────────┘
         │
         ↓
┌──────────────────┐
│ Autonomous Mgr   │
│ (Waypoint Gen)   │
└────────┬─────────┘
         │
         ↓
┌──────────────────┐
│ MAVLink Interface│
│ (Flight Control) │
└────────┬─────────┘
         │
         ↓
   ┌─────────────┐
   │ CubeOrange  │
   │ (Autopilot) │
   └─────────────┘
```

## Prerequisites

All autonomous modes require:
1. Drone armed
2. Flight mode set to GUIDED
3. Target locked (tracking active)
4. Valid GPS coordinates for target

## Loiter Mode

Circular orbit around locked target at fixed standoff distance.

### Parameters

```yaml
autonomous:
  standoff_distance: 50.0        # Orbit radius (meters)
  loiter_altitude_offset: 0.0    # Altitude change (meters)
  loiter_speed: 5.0              # Orbit speed (m/s)
  waypoint_update_interval: 2.0  # Update rate (seconds)
```

### Algorithm

**1. Initialize**
- Record current altitude as orbit altitude
- Calculate initial bearing to target
- Enter orbit at current position

**2. Update Loop (every 2 seconds)**

Calculate next orbit position:

```python
# Current position on orbit
current_bearing = bearing(drone_pos, target_pos)

# Orbital angular velocity
orbit_circumference = 2 * π * standoff_distance
orbit_period = orbit_circumference / loiter_speed
angular_velocity = 360° / orbit_period

# Advance angle
angle_increment = angular_velocity * update_interval
next_bearing = (current_bearing + angle_increment) % 360

# Calculate next waypoint
next_lat, next_lon = destination_point(
    target_lat, target_lon,
    next_bearing,
    standoff_distance
)

# Send waypoint
mavlink.goto_position(next_lat, next_lon, orbit_altitude)
```

**3. Behavior**
- Maintains constant altitude
- Smooth circular path
- Continuous target observation
- Adapts if target moves slightly

### Use Cases

- Stationary target observation
- Area surveillance
- Maintaining visual contact
- Photogrammetry (structure from motion)

## Follow Mode

Actively chase moving target maintaining standoff distance.

### Parameters

```yaml
autonomous:
  standoff_distance: 50.0           # Follow distance (meters)
  follow_speed: 8.0                 # Max speed (m/s)
  follow_altitude_mode: maintain    # maintain or match
  min_target_movement: 5.0          # Movement threshold (meters)
  waypoint_update_interval: 2.0     # Update rate (seconds)
```

### Algorithm

**1. Initialize**
- Record target position
- Calculate initial follow position

**2. Update Loop (every 2 seconds)**

Check if target moved:

```python
# Calculate movement
distance_moved = haversine(last_target_pos, current_target_pos)

if distance_moved < min_target_movement:
    return  # Target stationary, skip update

# Calculate follow position
# Position drone at standoff distance opposite drone bearing
bearing_to_target = bearing(drone_pos, target_pos)
follow_bearing = (bearing_to_target + 180) % 360

follow_lat, follow_lon = destination_point(
    target_lat, target_lon,
    follow_bearing,
    standoff_distance
)

# Altitude handling
if follow_altitude_mode == 'maintain':
    follow_alt = current_alt
else:
    # Could estimate target altitude from size/distance
    follow_alt = current_alt

# Send waypoint
mavlink.goto_position(follow_lat, follow_lon, follow_alt)

# Update last position
last_target_pos = current_target_pos
```

**3. Behavior**
- Maintains standoff distance behind target
- Updates position when target moves >5m
- Smooth pursuit trajectory
- Speed limited by follow_speed

### Use Cases

- Vehicle tracking
- Person following
- Convoy escort
- Dynamic target observation

## GPS Math Functions

### Bearing Calculation

Calculate compass bearing between two GPS points:

```python
def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate bearing from point 1 to point 2.

    Returns: Bearing in degrees (0-360)
    """
    lat1_rad = radians(lat1)
    lat2_rad = radians(lat2)
    dlon_rad = radians(lon2 - lon1)

    y = sin(dlon_rad) * cos(lat2_rad)
    x = cos(lat1_rad) * sin(lat2_rad) - \
        sin(lat1_rad) * cos(lat2_rad) * cos(dlon_rad)

    bearing_rad = atan2(y, x)
    bearing_deg = degrees(bearing_rad)

    return (bearing_deg + 360) % 360
```

### Distance Calculation

Haversine formula for great circle distance:

```python
def calculate_distance(lat1, lon1, lat2, lon2):
    """
    Calculate distance between two GPS points.

    Returns: Distance in meters
    """
    R = 6371000  # Earth radius (meters)

    lat1_rad = radians(lat1)
    lat2_rad = radians(lat2)
    dlat_rad = radians(lat2 - lat1)
    dlon_rad = radians(lon2 - lon1)

    a = sin(dlat_rad/2)**2 + \
        cos(lat1_rad) * cos(lat2_rad) * sin(dlon_rad/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))

    return R * c
```

### Destination Point

Calculate point at given distance and bearing:

```python
def calculate_destination(lat, lon, bearing, distance):
    """
    Calculate destination point from start point, bearing, and distance.

    Args:
        lat, lon: Start position (degrees)
        bearing: Bearing in degrees (0-360)
        distance: Distance in meters

    Returns: (lat, lon) in degrees
    """
    R = 6371000  # Earth radius (meters)

    lat_rad = radians(lat)
    lon_rad = radians(lon)
    bearing_rad = radians(bearing)

    lat2_rad = asin(
        sin(lat_rad) * cos(distance/R) +
        cos(lat_rad) * sin(distance/R) * cos(bearing_rad)
    )

    lon2_rad = lon_rad + atan2(
        sin(bearing_rad) * sin(distance/R) * cos(lat_rad),
        cos(distance/R) - sin(lat_rad) * sin(lat2_rad)
    )

    lat2 = degrees(lat2_rad)
    lon2 = degrees(lon2_rad)

    return (lat2, lon2)
```

## Safety Features

### Pre-Flight Checks

Before enabling autonomous mode:

```python
# 1. Check drone armed
if not drone.armed:
    return "Drone not armed"

# 2. Check flight mode
if drone.mode != "GUIDED":
    mavlink.set_mode("GUIDED")
    wait_for_mode("GUIDED", timeout=5)

# 3. Check target lock
if not tracking.is_tracking:
    return "No target locked"

# 4. Check GPS fix
if drone.gps_fix < 3:
    return "Insufficient GPS fix"

# 5. Check battery
if drone.battery_remaining < 30:
    return "Low battery"
```

### In-Flight Monitoring

Continuous safety checks:

```python
# 1. Target loss
if time.time() - target.last_seen > 5.0:
    disable_autonomous()
    return "Target lost"

# 2. GPS degradation
if drone.gps_satellites < 6:
    disable_autonomous()
    return "GPS degraded"

# 3. Battery critical
if drone.battery_remaining < 20:
    disable_autonomous()
    mavlink.set_mode("RTL")
    return "Battery critical - RTL"

# 4. Altitude limits
if drone.alt_rel > max_altitude:
    disable_autonomous()
    return "Altitude limit"

# 5. Range limits
if distance_from_home > max_range:
    disable_autonomous()
    return "Range limit"
```

### Emergency Procedures

Failsafe actions:

**Target Lost**
- Continue last heading for 5 seconds
- If target reappears, resume tracking
- Else, enter LOITER mode at current position

**GPS Loss**
- Switch to STABILIZE mode
- Maintain altitude and attitude
- Wait for GPS recovery

**Battery Critical**
- Disable autonomous mode
- Switch to RTL (Return to Launch)
- Land at home position

**RC Override**
- Pilot takes manual control
- Disable autonomous mode
- Return control to pilot

## Performance Tuning

### Orbit Smoothness

Adjust update interval:
- Faster updates (1s): Smoother orbit, more CPU
- Slower updates (3s): Choppy orbit, less CPU

Adjust orbit speed:
- Faster speed: Larger angle increments
- Slower speed: Smoother motion

### Follow Responsiveness

Adjust movement threshold:
- Lower threshold (2m): More responsive, more updates
- Higher threshold (10m): Less responsive, fewer updates

Adjust follow distance:
- Shorter distance: Tighter following
- Longer distance: More buffer

### Waypoint Acceptance

MAVLink waypoint acceptance radius:
- Smaller radius: More precise, may oscillate
- Larger radius: Less precise, smoother flight

## Testing

### Simulation (SITL)

Test with ArduPilot SITL:

```bash
# Start SITL
sim_vehicle.py -v ArduCopter --console --map

# Connect DroneBrain
# Edit config.yaml: connection_string: "udp:127.0.0.1:14550"
./start.sh

# Arm and takeoff in SITL
arm throttle
mode guided
takeoff 50

# Test autonomous modes via web UI
```

### Ground Test

Test on ground with motors disarmed:

```bash
# Start system
./start.sh

# Arm in GUIDED mode (motors won't spin if not calibrated)
# Test waypoint generation
# Verify GPS calculations
# Check safety interlocks
```

### Flight Test

Incremental flight testing:

**Test 1: Manual Flight**
- Fly manually in LOITER mode
- Verify GPS position
- Check battery, telemetry

**Test 2: Lock and Track**
- Lock onto stationary object
- Verify gimbal tracks correctly
- Check GPS calculation accuracy

**Test 3: Loiter Mode**
- Enable loiter mode at low altitude (10m)
- Small orbit radius (20m)
- Verify circular path
- Monitor for oscillations

**Test 4: Follow Mode**
- Lock onto slow-moving vehicle
- Enable follow mode
- Verify pursuit behavior
- Check standoff distance maintained

**Test 5: Full Mission**
- Higher altitude (50m)
- Larger orbit (50m)
- Longer duration
- Verify all safety features

## API Reference

### Enable Loiter Mode

```python
from src.autonomous.autonomous_manager import AutonomousMode

# Set mode
autonomous_manager.set_mode(AutonomousMode.LOITER)

# Check status
status = autonomous_manager.get_status()
print(f"Mode: {status['mode']}")
print(f"Standoff: {status['standoff_distance']}m")
```

### Enable Follow Mode

```python
autonomous_manager.set_mode(AutonomousMode.FOLLOW)
```

### Disable Autonomous

```python
autonomous_manager.set_mode(AutonomousMode.DISABLED)
```

### Update Standoff Distance

```python
# Update in config
config['autonomous']['standoff_distance'] = 75.0

# Restart autonomous manager
autonomous_manager = AutonomousManager(state, mavlink, config)
```

## Troubleshooting

### Drone Won't Enter GUIDED Mode

- Check flight mode switch on RC transmitter
- Verify GUIDED mode is enabled in autopilot params
- Check for pre-arm errors

### Waypoints Not Accepted

- Increase waypoint acceptance radius (WPNAV_RADIUS)
- Check GPS HDOP (should be <2.0)
- Verify MAVLink connection quality

### Erratic Flight Path

- Reduce waypoint update rate
- Increase standoff distance
- Check wind conditions
- Verify GPS position accuracy

### Target Tracking Unstable

- Improve lighting conditions
- Reduce detection confidence threshold
- Enable Kalman filtering for position
- Increase min_target_movement threshold
