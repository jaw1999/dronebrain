# TAK Integration Guide

DroneBrain includes integration with Team Awareness Kit (TAK) systems for tactical situational awareness. Detected and tracked objects are automatically published as Cursor on Target (CoT) XML messages to TAK servers.

## Overview

**TAK (Team Awareness Kit)** is a tactical situational awareness platform used by:
- Military operations
- Law enforcement
- Search and rescue teams
- Emergency response
- Disaster management

**CoT (Cursor on Target)** is the XML-based message format used by TAK systems to share entity positions, tracks, and events.

## Features

- **Automatic Publishing**: Tracked objects are automatically published to TAK server
- **Geolocation**: Uses Phase 3 geolocation to calculate GPS coordinates
- **CoT Generation**: Generates compliant CoT v2.0 XML messages
- **Reliable Delivery**: TCP connection with automatic reconnection
- **Configurable**: All parameters configurable via YAML config
- **Multiple Object Types**: Supports persons, vehicles, aircraft, boats, etc.

## Architecture

```
┌─────────────────┐
│  Vision System  │
│   (Detection)   │
└────────┬────────┘
         │
         ↓
┌─────────────────┐
│  Geolocator     │
│ (GPS Coords)    │
└────────┬────────┘
         │
         ↓
┌─────────────────┐
│  CoT Generator  │
│  (XML Messages) │
└────────┬────────┘
         │
         ↓
┌─────────────────┐
│  TAK Interface  │
│   (TCP/UDP)     │
└────────┬────────┘
         │
         ↓
   ┌─────────────┐
   │ TAK Server  │
   │  (8087/tcp) │
   └─────────────┘
```

## Configuration

Edit `config/config.yaml`:

```yaml
tak:
  enabled: true                       # Enable TAK integration
  server_ip: "192.168.1.100"          # TAK server IP
  server_port: 8087                   # TAK server port (8087 for TCP CoT)
  protocol: "tcp"                     # "tcp" or "udp"
  callsign: "DRONEBRAIN-1"            # This drone's callsign
  publish_interval: 1.0               # Publish every N seconds
  stale_minutes: 5                    # Minutes until message becomes stale
  default_ce: 10.0                    # Circular error (meters)
  default_le: 10.0                    # Linear error (meters)
  reconnect_interval: 5.0             # Reconnect delay (seconds)
  queue_size: 100                     # Max queued messages
```

## CoT Message Format

### Detection CoT Message

Each tracked object is published with:

```xml
<event version="2.0" uid="DRONEBRAIN-1-42" type="a-.-G-E-V-C"
       how="m-g" time="..." start="..." stale="...">
  <point lat="34.123456" lon="-118.654321" hae="100.0"
         ce="10.0" le="10.0"/>
  <detail>
    <contact callsign="PERSON-42"/>
    <remarks>Detected by DRONEBRAIN-1 | Class: person |
             Confidence: 0.95 | Track ID: 42</remarks>
    <track course="0.0" speed="0.0"/>
  </detail>
</event>
```

### CoT Type Codes

Object classes are mapped to CoT type codes:

| Object Class | CoT Type      | Description                    |
|--------------|---------------|--------------------------------|
| person       | a-.-G-E-V-C   | Ground/Equipment/Vehicle/Civ   |
| car          | a-.-G-E-V-C   | Ground/Equipment/Vehicle/Civ   |
| truck        | a-.-G-E-V-C   | Ground/Equipment/Vehicle/Civ   |
| boat         | a-.-G-E-S     | Ground/Equipment/Sea           |
| airplane     | a-f-A-C-F     | Air/Friendly/Fixed Wing        |
| helicopter   | a-f-A-C-H     | Air/Friendly/Helicopter        |
| (unknown)    | a-u-G         | Atom/Unknown/Ground            |

## TAK Server Options

### 1. TAK Server (Official)

The official TAK Server from TAK Product Center:
- Download: https://tak.gov
- Requires government authorization
- Supports CoT over TCP port 8087

### 2. FreeTAKServer (Open Source)

Free and open-source TAK server:
- GitHub: https://github.com/FreeTAKTeam/FreeTAKServer
- Documentation: https://freetakteam.github.io/FreeTAKServer-User-Docs/

Install:
```bash
pip install FreeTAKServer[all]
```

Run:
```bash
python -m FreeTAKServer.controllers.services.FTS
```

Default CoT port: 8087 (TCP)

### 3. TAK Server Docker

Run FreeTAKServer in Docker:
```bash
docker run -d \
  -p 8087:8087 \
  -p 8080:8080 \
  --name freetakserver \
  freetakteam/freetakserver:latest
```

## Testing

### 1. Test CoT Generation

```bash
python examples/tak_demo.py
# Select option 1: Test CoT message generation
```

This tests CoT XML generation without requiring a TAK server.

### 2. Test TAK Connection

```bash
python examples/tak_demo.py
# Select option 2: Test TAK server connection
```

This connects to a TAK server and sends a single test message.

### 3. Test Continuous Publishing

```bash
python examples/tak_demo.py
# Select option 3: Test continuous publishing
```

This continuously publishes simulated detections to TAK server.

## Usage in DroneBrain

Once configured, TAK integration runs automatically:

```python
from src.core.pipeline_manager import PipelineManager

# Initialize pipeline (TAK auto-configured from config.yaml)
pipeline = PipelineManager(config_path="config/config.yaml")
pipeline.initialize()

# Start system (TAK client starts automatically)
pipeline.start()

# Tracked objects are automatically published to TAK server
# No additional code needed!
```

## Viewing in TAK Clients

Published objects can be viewed in:

### ATAK (Android Team Awareness Kit)
- Install ATAK on Android device
- Configure server connection to TAK server IP
- View detected objects on map

### WinTAK (Windows TAK)
- Install WinTAK on Windows PC
- Configure server connection
- View detected objects on map

### iTAK (iOS TAK)
- Install iTAK on iOS device
- Configure server connection
- View detected objects on map

## Troubleshooting

### Connection Refused

**Problem**: Cannot connect to TAK server

**Solutions**:
1. Verify TAK server is running: `netstat -an | grep 8087`
2. Check firewall allows port 8087
3. Verify correct IP address in config
4. Check TAK server logs for errors

### Messages Not Appearing

**Problem**: Messages sent but not visible in TAK client

**Solutions**:
1. Verify TAK client is connected to same server
2. Check CoT message format is valid (use demo script option 1)
3. Verify geolocation is calculating valid GPS coordinates
4. Check TAK server accepts anonymous connections

### High CPU Usage

**Problem**: TAK integration uses excessive CPU

**Solutions**:
1. Increase `publish_interval` (e.g., 2.0 seconds)
2. Reduce number of tracked objects
3. Use UDP instead of TCP for lower overhead

## Advanced Configuration

### Custom CoT Types

Edit [cot_generator.py](../src/interfaces/cot_generator.py) to customize CoT type mappings:

```python
COT_TYPES = {
    "person": "a-.-G-E-V-C",  # Civilian
    "car": "a-.-G-E-V-C",     # Civilian vehicle
    # Add custom mappings here
}
```

### Publishing Only Specific Classes

Modify vision processing loop to filter published objects:

```python
# Only publish persons and vehicles
if detection.class_name in ["person", "car", "truck"]:
    cot_message = self.cot_generator.generate_detection_cot(...)
    self.tak_client.send_cot(cot_message)
```

### Adding Metadata

Extend CoT messages with custom metadata in [cot_generator.py](../src/interfaces/cot_generator.py):

```python
# Add custom detail elements
detail = ET.SubElement(event, "detail")

# Add sensor info
sensor = ET.SubElement(detail, "sensor")
sensor.set("type", "EO/IR")
sensor.set("fov", "30")
```

## Integration with Other Systems

### ATAK Plugins

Create ATAK plugins that consume DroneBrain CoT data:
- Display detection overlays
- Trigger alerts for specific object types
- Record track history

### TAK Server APIs

Use TAK Server REST APIs to:
- Query historical tracks
- Generate alerts
- Export track data

### Mission Planning

Integrate with mission planning systems:
- Feed detections into route planning
- Avoid detected obstacles
- Update threat assessments

## Performance

Typical performance metrics:

- **CoT Generation**: <1ms per object
- **Network Latency**: 5-20ms (LAN), 50-200ms (WAN)
- **Publish Rate**: 1-10 Hz configurable
- **Queue Depth**: 100 messages default
- **CPU Overhead**: <5% on Jetson Orin Nano

## Security Considerations

### Network Security

- Use TLS/SSL for production deployments
- Configure TAK server authentication
- Use VPN for remote connections
- Restrict TAK server port access with firewall

### Data Classification

- Mark CoT messages with appropriate classification
- Encrypt sensitive detections
- Use TAK server access controls
- Audit CoT message logs

### Operational Security

- Use anonymous callsigns in hostile environments
- Disable TAK in RF-denied operations
- Implement CoT rate limiting to prevent tracking
- Sanitize metadata before publishing

## API Reference

### CoTGenerator

```python
from src.interfaces.cot_generator import CoTGenerator

generator = CoTGenerator(
    source_callsign="DRONEBRAIN",
    stale_minutes=5,
    default_ce=10.0,
    default_le=10.0,
)

# Generate detection CoT
cot_xml = generator.generate_detection_cot(
    detection=detection,
    lat=34.123456,
    lon=-118.654321,
    alt_msl=100.0,
)

# Generate platform CoT
platform_cot = generator.generate_sensor_platform_cot(
    lat=34.123456,
    lon=-118.654321,
    alt_msl=150.0,
    heading=45.0,
)
```

### TAKInterface

```python
from src.interfaces.tak_interface import TAKInterface

tak = TAKInterface(
    server_ip="192.168.1.100",
    server_port=8087,
    protocol="tcp",
)

# Start connection
tak.start()

# Send CoT message
tak.send_cot(cot_xml_message)

# Get stats
stats = tak.get_stats()

# Stop connection
tak.stop()
```

## References

- [Cursor on Target (CoT) Specification](https://www.mitre.org/sites/default/files/pdf/09_4937.pdf)
- [TAK Product Center](https://tak.gov)
- [FreeTAKServer Documentation](https://freetakteam.github.io/FreeTAKServer-User-Docs/)
- [ATAK-Civ Downloads](https://tak.gov/products/atak-civ)
