"""
CoT (Cursor on Target) Message Generator

Generates CoT XML messages for tracked objects to send to TAK servers.
Follows CoT Version 2.0 specification.
"""

import logging
from datetime import datetime, timedelta
from typing import Optional
from xml.etree import ElementTree as ET

from src.vision.yolo_detector import Detection

logger = logging.getLogger(__name__)


class CoTGenerator:
    """
    Generate Cursor on Target (CoT) XML messages for tracked objects.

    CoT messages are used by TAK (Team Awareness Kit) systems for
    tactical situational awareness.

    Example:
        >>> generator = CoTGenerator(source_callsign="DRONEBRAIN-1")
        >>> cot_xml = generator.generate_detection_cot(
        ...     detection=detection,
        ...     lat=34.123456,
        ...     lon=-118.123456,
        ...     alt_msl=100.0
        ... )
    """

    # CoT type codes for different object classes
    COT_TYPES = {
        "person": "a-.-G-E-V-C",  # Ground/Equipment/Vehicle/Civilian (person on ground)
        "car": "a-.-G-E-V-C",  # Ground/Equipment/Vehicle/Civilian
        "truck": "a-.-G-E-V-C",  # Ground/Equipment/Vehicle/Civilian
        "bus": "a-.-G-E-V-C",  # Ground/Equipment/Vehicle/Civilian
        "motorcycle": "a-.-G-E-V-C",  # Ground/Equipment/Vehicle/Civilian
        "bicycle": "a-.-G-E-V-C",  # Ground/Equipment/Vehicle/Civilian
        "boat": "a-.-G-E-S",  # Ground/Equipment/Sea
        "airplane": "a-f-A-C-F",  # Air/Friendly/Fixed Wing
        "helicopter": "a-f-A-C-H",  # Air/Friendly/Helicopter
        "default": "a-u-G",  # Atom/Unknown/Ground
    }

    def __init__(
        self,
        source_callsign: str = "DRONEBRAIN",
        stale_minutes: int = 5,
        default_ce: float = 10.0,
        default_le: float = 10.0,
    ):
        """
        Initialize CoT generator.

        Args:
            source_callsign: Callsign for this drone/system
            stale_minutes: Minutes until CoT message becomes stale
            default_ce: Default circular error in meters
            default_le: Default linear error in meters
        """
        self.source_callsign = source_callsign
        self.stale_minutes = stale_minutes
        self.default_ce = default_ce
        self.default_le = default_le

    def generate_detection_cot(
        self,
        detection: Detection,
        lat: float,
        lon: float,
        alt_msl: Optional[float] = None,
        circular_error: Optional[float] = None,
        linear_error: Optional[float] = None,
    ) -> str:
        """
        Generate CoT XML message for a detected object.

        Args:
            detection: Detection object with track ID, class, bbox, etc.
            lat: Latitude of detected object (decimal degrees)
            lon: Longitude of detected object (decimal degrees)
            alt_msl: Altitude MSL in meters (optional)
            circular_error: Circular error in meters (optional)
            linear_error: Linear error in meters (optional)

        Returns:
            CoT XML message as string
        """
        # Generate timestamps
        now = datetime.utcnow()
        stale = now + timedelta(minutes=self.stale_minutes)

        time_str = now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
        start_str = time_str
        stale_str = stale.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"

        # Generate unique ID
        uid = f"{self.source_callsign}-{detection.track_id}"

        # Get CoT type for this class
        cot_type = self.COT_TYPES.get(detection.class_name, self.COT_TYPES["default"])

        # Use provided errors or defaults
        ce = circular_error if circular_error is not None else self.default_ce
        le = linear_error if linear_error is not None else self.default_le
        hae = alt_msl if alt_msl is not None else 0.0

        # Create event element
        event = ET.Element("event")
        event.set("version", "2.0")
        event.set("uid", uid)
        event.set("type", cot_type)
        event.set("how", "m-g")  # machine-generated
        event.set("time", time_str)
        event.set("start", start_str)
        event.set("stale", stale_str)

        # Add point element
        point = ET.SubElement(event, "point")
        point.set("lat", f"{lat:.6f}")
        point.set("lon", f"{lon:.6f}")
        point.set("hae", f"{hae:.1f}")
        point.set("ce", f"{ce:.1f}")
        point.set("le", f"{le:.1f}")

        # Add detail element with extra info
        detail = ET.SubElement(event, "detail")

        # Contact info
        contact = ET.SubElement(detail, "contact")
        contact.set("callsign", f"{detection.class_name.upper()}-{detection.track_id}")

        # Remarks with detection info
        remarks = ET.SubElement(detail, "remarks")
        remarks.text = (
            f"Detected by {self.source_callsign} | "
            f"Class: {detection.class_name} | "
            f"Confidence: {detection.confidence:.2f} | "
            f"Track ID: {detection.track_id}"
        )

        # Track metadata
        track = ET.SubElement(detail, "track")
        track.set("course", "0.0")  # Unknown course
        track.set("speed", "0.0")  # Unknown speed

        # Convert to XML string
        xml_str = ET.tostring(event, encoding="unicode", method="xml")

        return xml_str

    def generate_sensor_platform_cot(
        self,
        lat: float,
        lon: float,
        alt_msl: float,
        heading: Optional[float] = None,
    ) -> str:
        """
        Generate CoT message for the sensor platform (drone) itself.

        Args:
            lat: Drone latitude (decimal degrees)
            lon: Drone longitude (decimal degrees)
            alt_msl: Drone altitude MSL in meters
            heading: Drone heading in degrees (optional)

        Returns:
            CoT XML message as string
        """
        # Generate timestamps
        now = datetime.utcnow()
        stale = now + timedelta(minutes=self.stale_minutes)

        time_str = now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
        start_str = time_str
        stale_str = stale.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"

        # UID for this platform
        uid = self.source_callsign

        # CoT type for friendly UAV
        cot_type = "a-f-A-C-F-q"  # Air/Friendly/Fixed Wing/RPV/Drone

        # Create event element
        event = ET.Element("event")
        event.set("version", "2.0")
        event.set("uid", uid)
        event.set("type", cot_type)
        event.set("how", "m-g")
        event.set("time", time_str)
        event.set("start", start_str)
        event.set("stale", stale_str)

        # Add point element
        point = ET.SubElement(event, "point")
        point.set("lat", f"{lat:.6f}")
        point.set("lon", f"{lon:.6f}")
        point.set("hae", f"{alt_msl:.1f}")
        point.set("ce", "5.0")  # Assume GPS accuracy
        point.set("le", "5.0")

        # Add detail element
        detail = ET.SubElement(event, "detail")

        # Contact info
        contact = ET.SubElement(detail, "contact")
        contact.set("callsign", self.source_callsign)

        # Remarks
        remarks = ET.SubElement(detail, "remarks")
        remarks.text = "DroneBrain UAV Platform"

        # Track info
        if heading is not None:
            track = ET.SubElement(detail, "track")
            track.set("course", f"{heading:.1f}")
            track.set("speed", "0.0")

        # Convert to XML string
        xml_str = ET.tostring(event, encoding="unicode", method="xml")

        return xml_str
