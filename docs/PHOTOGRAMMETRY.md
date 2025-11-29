# Photogrammetry and Geolocation

DroneBrain calculates GPS coordinates of detected targets using photogrammetry - the science of making measurements from photographs.

## Overview

Given:
- Drone GPS position (lat, lon, alt)
- Drone attitude (roll, pitch, yaw)
- Gimbal angles (pitch, yaw)
- Camera intrinsics (focal length, sensor size)
- Target pixel coordinates in frame

Calculate:
- Target GPS coordinates (lat, lon)

## Mathematical Model

### Coordinate Systems

**1. Camera Frame (C)**
- Origin: Camera optical center
- X-axis: Right
- Y-axis: Down
- Z-axis: Forward (optical axis)

**2. Gimbal Frame (G)**
- Rotated by gimbal pitch and yaw

**3. Body Frame (B)**
- Drone body axes
- Roll, pitch, yaw relative to NED

**4. NED Frame (North-East-Down)**
- Navigation frame
- X-axis: North
- Y-axis: East
- Z-axis: Down

**5. ECEF Frame (Earth-Centered Earth-Fixed)**
- Global Cartesian coordinates

### Transform Chain

```
Camera → Gimbal → Body → NED → ECEF → GPS
```

## Implementation

### 1. Pixel to Camera Ray

Convert pixel coordinates to normalized camera ray:

```python
# Camera intrinsics
fx = focal_length_mm * image_width / sensor_width_mm
fy = focal_length_mm * image_height / sensor_height_mm
cx = image_width / 2
cy = image_height / 2

# Pixel to normalized coordinates
x_norm = (pixel_x - cx) / fx
y_norm = (pixel_y - cy) / fy

# Ray in camera frame (unit vector)
ray_camera = np.array([x_norm, y_norm, 1.0])
ray_camera = ray_camera / np.linalg.norm(ray_camera)
```

### 2. Camera to Gimbal Transform

Rotate by gimbal pitch and yaw:

```python
# Gimbal pitch rotation (around Y-axis)
R_pitch = np.array([
    [cos(pitch), 0, sin(pitch)],
    [0, 1, 0],
    [-sin(pitch), 0, cos(pitch)]
])

# Gimbal yaw rotation (around Z-axis)
R_yaw = np.array([
    [cos(yaw), -sin(yaw), 0],
    [sin(yaw), cos(yaw), 0],
    [0, 0, 1]
])

# Combined rotation
R_gimbal = R_yaw @ R_pitch
ray_gimbal = R_gimbal @ ray_camera
```

### 3. Gimbal to Body Transform

Account for gimbal mounting offset and body frame:

```python
# Typically gimbal is mounted with Z down, X forward
# Adjust for specific mounting configuration
R_gimbal_to_body = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])

ray_body = R_gimbal_to_body @ ray_gimbal
```

### 4. Body to NED Transform

Rotate by drone attitude (roll, pitch, yaw):

```python
# Roll rotation (around X-axis)
R_roll = np.array([
    [1, 0, 0],
    [0, cos(roll), -sin(roll)],
    [0, sin(roll), cos(roll)]
])

# Pitch rotation (around Y-axis)
R_pitch = np.array([
    [cos(pitch), 0, sin(pitch)],
    [0, 1, 0],
    [-sin(pitch), 0, cos(pitch)]
])

# Yaw rotation (around Z-axis)
R_yaw = np.array([
    [cos(yaw), -sin(yaw), 0],
    [sin(yaw), cos(yaw), 0],
    [0, 0, 1]
])

# Combined rotation (ZYX Euler angles)
R_body_to_ned = R_yaw @ R_pitch @ R_roll
ray_ned = R_body_to_ned @ ray_body
```

### 5. Ray-Ground Intersection

Assume target is on ground (zero elevation):

```python
# Drone position in NED (origin at takeoff point)
drone_ned = np.array([north, east, -altitude])

# Ground plane: z = -ground_elevation
# Ray equation: P = drone_ned + t * ray_ned
# Solve for t when z = -ground_elevation

t = (-ground_elevation - drone_ned[2]) / ray_ned[2]

# Intersection point in NED
target_ned = drone_ned + t * ray_ned
```

### 6. NED to GPS Coordinates

Convert NED offset to GPS coordinates:

```python
# Earth radius
R_earth = 6371000  # meters

# Convert NED to lat/lon offsets
d_lat = target_ned[0] / R_earth  # North offset
d_lon = target_ned[1] / (R_earth * cos(drone_lat))  # East offset

# Target GPS
target_lat = drone_lat + degrees(d_lat)
target_lon = drone_lon + degrees(d_lon)
```

## Error Sources

### 1. Altitude Measurement Error
- ±1m GPS altitude error → ±1m ground position error
- Use barometric altitude for better accuracy

### 2. Attitude Estimation Error
- ±1° attitude error → ±1.7% range error
- At 100m range: ±1.7m position error

### 3. Gimbal Angle Error
- ±0.1° gimbal error → ±0.17% range error
- Calibrate gimbal regularly

### 4. Camera Calibration Error
- Focal length error affects range estimate
- Sensor size error affects angular calculation

### 5. Ground Elevation Assumption
- Assumes flat ground at MSL
- Add terrain elevation data for accuracy

### 6. Pixel Localization Error
- ±1 pixel error at 1920x1080
- Use bounding box center for stability

## Accuracy Analysis

Typical error budget at 100m range:

| Error Source | Magnitude | Position Error |
|--------------|-----------|----------------|
| GPS altitude | ±1m | ±1.0m |
| Attitude | ±1° | ±1.7m |
| Gimbal | ±0.1° | ±0.17m |
| Pixel | ±2px | ±0.2m |
| **Total (RSS)** | - | **±2.0m** |

At 50m range: ±1.0m
At 200m range: ±4.0m

## Optimization Techniques

### 1. Kalman Filtering

Track target position over time:

```python
# State: [lat, lon, lat_vel, lon_vel]
# Measurement: [lat_meas, lon_meas]

# Predict
x_pred = F @ x_prev  # State transition
P_pred = F @ P_prev @ F.T + Q  # Covariance

# Update
y = z - H @ x_pred  # Innovation
K = P_pred @ H.T @ inv(H @ P_pred @ H.T + R)  # Kalman gain
x = x_pred + K @ y
P = (I - K @ H) @ P_pred
```

### 2. Multi-Frame Averaging

Average measurements across frames:

```python
# Exponential moving average
lat_filtered = alpha * lat_new + (1 - alpha) * lat_prev
lon_filtered = alpha * lon_new + (1 - alpha) * lon_prev

# alpha = 0.3 for 30% new, 70% history
```

### 3. Terrain Compensation

Use DEM (Digital Elevation Model):

```python
# Query terrain elevation at approximate position
ground_elevation = get_terrain_elevation(approx_lat, approx_lon)

# Iterate to refine
for i in range(3):
    target_ned = ray_ground_intersection(ground_elevation)
    target_gps = ned_to_gps(target_ned)
    ground_elevation = get_terrain_elevation(target_gps)
```

## Validation

### Test Scenarios

**1. Known Position Targets**
- Place markers at surveyed GPS coordinates
- Measure calculated vs actual position
- Record errors at various ranges

**2. Moving Vehicle Tracking**
- Track vehicle with onboard GPS logger
- Compare DroneBrain position to GPS log
- Validate dynamic accuracy

**3. Altitude Sensitivity**
- Vary drone altitude (10m to 100m)
- Measure position error vs altitude
- Validate error scaling

### Ground Truth Comparison

```python
# Calculate error metrics
error_distance = haversine(calculated_gps, ground_truth_gps)
error_north = (calculated_lat - truth_lat) * R_earth
error_east = (calculated_lon - truth_lon) * R_earth * cos(truth_lat)

# Statistics
mean_error = np.mean(errors)
std_error = np.std(errors)
rms_error = np.sqrt(np.mean(errors**2))
cep_50 = np.percentile(errors, 50)  # Circular Error Probable
```

## API Reference

### Geolocator Class

```python
from src.tracking.geolocator import Geolocator

geolocator = Geolocator(
    focal_length_mm=21.0,
    sensor_width_mm=6.3,
    sensor_height_mm=4.7,
    image_width=1920,
    image_height=1080,
)

# Calculate target GPS
target_gps = geolocator.pixel_to_gps(
    pixel_x=960,
    pixel_y=540,
    drone_lat=34.123456,
    drone_lon=-118.654321,
    drone_alt_msl=150.0,
    drone_roll=0.0,
    drone_pitch=0.0,
    drone_yaw=45.0,
    gimbal_pitch=-30.0,
    gimbal_yaw=0.0,
    ground_elevation=0.0,
)

# Returns: (latitude, longitude, distance_m)
```

## References

- Photogrammetric Computer Vision (Förstner & Wrobel)
- Multiple View Geometry (Hartley & Zisserman)
- Coordinate Systems for Aerial Photogrammetry (ASPRS)
- MAVLink Camera Protocol Specification
