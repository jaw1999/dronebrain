# API Documentation

DroneBrain provides REST and WebSocket APIs for control and telemetry.

## Base URL

```
http://<jetson-ip>:8080
```

Default: `http://localhost:8080`

## REST API

### System Status

#### GET /api/health

Service health check.

**Response:**
```json
{
  "status": "ok",
  "service": "dronebrain-web-ui"
}
```

#### GET /api/status

System status.

**Response:**
```json
{
  "status": "running",
  "drone": {...},
  "gimbal": {...},
  "camera": {...},
  "detections": 3
}
```

### Drone Control

#### GET /api/telemetry/drone

Get drone telemetry.

**Response:**
```json
{
  "connected": true,
  "armed": true,
  "mode": "GUIDED",
  "position": {
    "latitude": 34.123456,
    "longitude": -118.654321,
    "altitude": 150.5,
    "altitude_relative": 50.2
  },
  "attitude": {
    "roll": 0.5,
    "pitch": -2.1,
    "yaw": 45.0
  },
  "velocity": {
    "vx": 5.2,
    "vy": 1.1,
    "vz": -0.1
  },
  "battery": 75,
  "gps_fix": 3,
  "satellites": 12
}
```

### Gimbal Control

#### GET /api/telemetry/gimbal

Get gimbal state.

**Response:**
```json
{
  "connected": true,
  "yaw": 15.0,
  "pitch": -30.5,
  "roll": 0.0,
  "locked": false
}
```

#### POST /api/gimbal/center

Center gimbal (0° pitch, 0° yaw).

**Response:**
```json
{
  "success": true
}
```

#### POST /api/gimbal/attitude

Set gimbal attitude.

**Request:**
```json
{
  "yaw": 0.0,
  "pitch": -45.0
}
```

Pitch: -90° (down) to +25° (up)
Yaw: -135° to +135°

**Response:**
```json
{
  "success": true
}
```

#### POST /api/gimbal/move

Move gimbal relative to current position.

**Request:**
```json
{
  "yaw_rate": 10.0,
  "pitch_rate": -5.0
}
```

**Response:**
```json
{
  "success": true
}
```

#### POST /api/gimbal/zoom

Set zoom level.

**Request:**
```json
{
  "zoom_level": 5.0
}
```

Zoom: 1.0 to 30.0

**Response:**
```json
{
  "success": true
}
```

### Target Tracking

#### GET /api/tracking/status

Get tracking state.

**Response:**
```json
{
  "enabled": true,
  "state": "TRACKING",
  "target": {
    "track_id": 42,
    "class_name": "person",
    "confidence": 0.95,
    "position": {
      "latitude": 34.123456,
      "longitude": -118.654321,
      "distance": 75.5
    }
  }
}
```

#### POST /api/tracking/start/{track_id}

Lock onto track.

**Response:**
```json
{
  "success": true,
  "track_id": 42
}
```

#### POST /api/tracking/stop

Stop tracking.

**Response:**
```json
{
  "success": true
}
```

### Autonomous Flight

#### GET /api/autonomous/status

Get autonomous mode status.

**Response:**
```json
{
  "mode": "loiter",
  "enabled": true,
  "standoff_distance": 50.0,
  "target_locked": true,
  "target_info": {
    "track_id": 42,
    "latitude": 34.123456,
    "longitude": -118.654321,
    "distance": 48.5,
    "last_seen": 1234567890.5
  }
}
```

#### POST /api/autonomous/loiter

Enable loiter mode (circular orbit).

**Response:**
```json
{
  "success": true,
  "mode": "loiter"
}
```

Requirements:
- Drone armed
- GUIDED mode
- Target locked

#### POST /api/autonomous/follow

Enable follow mode (chase target).

**Response:**
```json
{
  "success": true,
  "mode": "follow"
}
```

#### POST /api/autonomous/disable

Disable autonomous modes.

**Response:**
```json
{
  "success": true,
  "mode": "disabled"
}
```

#### POST /api/autonomous/standoff

Set standoff distance.

**Request:**
```json
{
  "distance": 75.0
}
```

Distance: 10.0 to 500.0 meters

**Response:**
```json
{
  "success": true,
  "standoff_distance": 75.0
}
```

### Configuration

#### GET /api/config

Get current configuration.

**Response:**
```json
{
  "drone": {...},
  "camera": {...},
  "vision": {...},
  "tracker": {...},
  "streaming": {...},
  "tak": {...},
  "autonomous": {...},
  "web": {...}
}
```

#### POST /api/config

Update configuration (requires restart).

**Request:**
```json
{
  "vision": {
    "confidence": 0.3
  }
}
```

**Response:**
```json
{
  "success": true,
  "message": "Configuration updated. Restart required."
}
```

## WebSocket API

### Connection

```javascript
const ws = new WebSocket('ws://localhost:8080/ws');
```

### Telemetry Stream

Subscribe to real-time telemetry (10Hz).

**Received Message:**
```json
{
  "type": "telemetry",
  "timestamp": 1234567890.5,
  "drone": {
    "armed": true,
    "mode": "GUIDED",
    "lat": 34.123456,
    "lon": -118.654321,
    "alt_msl": 150.5,
    "alt_rel": 50.2,
    "heading": 45.0,
    "groundspeed": 5.2,
    "roll": 0.5,
    "pitch": -2.1,
    "yaw": 45.0,
    "battery_voltage": 16.8,
    "battery_remaining": 75
  },
  "gimbal": {
    "pitch": -30.5,
    "yaw": 15.0,
    "zoom": 1.0,
    "recording": false
  },
  "tracking": {
    "is_tracking": true,
    "locked_track_id": 42,
    "target_lat": 34.123456,
    "target_lon": -118.654321,
    "target_distance": 75.5
  },
  "autonomous": {
    "mode": "loiter",
    "enabled": true,
    "standoff_distance": 50.0
  },
  "detections": [
    {
      "track_id": 42,
      "class_name": "person",
      "confidence": 0.95,
      "bbox": [100, 200, 300, 400]
    }
  ]
}
```


## Error Responses

All endpoints return errors in this format:

```json
{
  "success": false,
  "error": "Error description"
}
```

HTTP Status Codes:
- `200` - Success
- `400` - Bad request (invalid parameters)
- `404` - Endpoint not found
- `500` - Internal server error
- `503` - Service unavailable (hardware disconnected)

## Rate Limits

- REST API: 100 requests/second
- WebSocket: No limit (telemetry at 10Hz)

## Authentication

Currently no authentication required.

For production:
- Use reverse proxy with auth (nginx, traefik)
- Implement API keys
- Use TLS/SSL

## Example Clients

### Python

```python
import requests

# Get telemetry
response = requests.get('http://localhost:8080/api/telemetry/drone')
data = response.json()
print(f"Altitude: {data['position']['altitude_relative']}m")

# Start tracking
requests.post('http://localhost:8080/api/tracking/start/42')

# Enable loiter
requests.post('http://localhost:8080/api/autonomous/loiter')
```

### JavaScript

```javascript
// REST API
fetch('http://localhost:8080/api/telemetry/drone')
  .then(r => r.json())
  .then(data => console.log('Altitude:', data.position.altitude_relative));

// WebSocket
const ws = new WebSocket('ws://localhost:8080/ws');
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  if (data.type === 'telemetry') {
    console.log('Battery:', data.drone.battery);
  }
};
```

### cURL

```bash
# Get status
curl http://localhost:8080/api/status

# Get drone telemetry
curl http://localhost:8080/api/telemetry/drone

# Start tracking
curl -X POST http://localhost:8080/api/tracking/start/42

# Enable loiter
curl -X POST http://localhost:8080/api/autonomous/loiter
```
