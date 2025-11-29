#!/usr/bin/env python3
"""
TAK Integration Demo

Demonstrates how to publish tracked objects to a TAK server for
tactical situational awareness.

This demo simulates tracked detections and publishes them as CoT
(Cursor on Target) messages to a TAK server.

Usage:
    python examples/tak_demo.py

Requirements:
    - TAK server running (e.g., TAK Server, FreeTAKServer, etc.)
    - Update config/config.yaml with TAK server IP and port
    - Set tak.enabled = true in config
"""

import time
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.interfaces.cot_generator import CoTGenerator
from src.interfaces.tak_interface import TAKInterface
from src.vision.yolo_detector import Detection


def test_cot_generation():
    """Test CoT message generation."""
    print("=" * 60)
    print("CoT Message Generation Test")
    print("=" * 60)

    # Create CoT generator
    generator = CoTGenerator(
        source_callsign="DRONEBRAIN-TEST",
        stale_minutes=5,
    )

    # Create a mock detection
    detection = Detection(
        bbox=(100, 100, 200, 200),
        confidence=0.95,
        class_id=0,
        class_name="person",
        track_id=42,
    )

    # Generate CoT message for detection at a GPS location
    print("\nGenerating CoT message for tracked person...")
    cot_xml = generator.generate_detection_cot(
        detection=detection,
        lat=34.123456,
        lon=-118.654321,
        alt_msl=100.0,
    )

    print("\nGenerated CoT XML:")
    print("-" * 60)
    print(cot_xml)
    print("-" * 60)

    # Generate CoT message for sensor platform
    print("\nGenerating CoT message for sensor platform (drone)...")
    platform_cot = generator.generate_sensor_platform_cot(
        lat=34.123456,
        lon=-118.654321,
        alt_msl=150.0,
        heading=45.0,
    )

    print("\nGenerated Platform CoT XML:")
    print("-" * 60)
    print(platform_cot)
    print("-" * 60)


def test_tak_connection():
    """Test connection to TAK server."""
    print("\n" + "=" * 60)
    print("TAK Server Connection Test")
    print("=" * 60)

    # Get TAK server details from user
    print("\nEnter TAK server details:")
    server_ip = input("  Server IP [127.0.0.1]: ").strip() or "127.0.0.1"
    server_port = input("  Server Port [8087]: ").strip() or "8087"
    server_port = int(server_port)
    protocol = input("  Protocol (tcp/udp) [tcp]: ").strip().lower() or "tcp"

    # Create TAK interface
    print(f"\nConnecting to TAK server at {server_ip}:{server_port} ({protocol.upper()})...")
    tak = TAKInterface(
        server_ip=server_ip,
        server_port=server_port,
        protocol=protocol,
    )

    if tak.connect():
        print("✓ Connected successfully!")

        # Send a test CoT message
        print("\nSending test CoT message...")
        generator = CoTGenerator(source_callsign="DRONEBRAIN-TEST")

        # Create mock detection
        detection = Detection(
            bbox=(100, 100, 200, 200),
            confidence=0.95,
            class_id=0,
            class_name="person",
            track_id=1,
        )

        # Generate and send CoT
        cot_xml = generator.generate_detection_cot(
            detection=detection,
            lat=34.123456,
            lon=-118.654321,
            alt_msl=100.0,
        )

        if tak.send_cot_immediate(cot_xml):
            print("✓ CoT message sent successfully!")
        else:
            print("✗ Failed to send CoT message")

        # Get stats
        stats = tak.get_stats()
        print(f"\nTAK Interface Stats:")
        print(f"  Connected: {stats['connected']}")
        print(f"  Messages sent: {stats['messages_sent']}")
        print(f"  Messages failed: {stats['messages_failed']}")
        print(f"  Bytes sent: {stats['bytes_sent']}")

        tak.disconnect()
        print("\nDisconnected from TAK server")

    else:
        print("✗ Failed to connect to TAK server")
        print("  Make sure TAK server is running and accessible")


def test_continuous_publishing():
    """Test continuous publishing to TAK server."""
    print("\n" + "=" * 60)
    print("Continuous TAK Publishing Test")
    print("=" * 60)

    # Get TAK server details
    print("\nEnter TAK server details:")
    server_ip = input("  Server IP [127.0.0.1]: ").strip() or "127.0.0.1"
    server_port = input("  Server Port [8087]: ").strip() or "8087"
    server_port = int(server_port)
    protocol = input("  Protocol (tcp/udp) [tcp]: ").strip().lower() or "tcp"

    # Create TAK interface and generator
    tak = TAKInterface(
        server_ip=server_ip,
        server_port=server_port,
        protocol=protocol,
    )

    generator = CoTGenerator(source_callsign="DRONEBRAIN-TEST")

    # Start TAK interface
    print(f"\nStarting TAK interface...")
    if not tak.start():
        print("✗ Failed to start TAK interface")
        return

    print("✓ TAK interface started")
    print("\nPublishing simulated detections to TAK server...")
    print("Press Ctrl+C to stop\n")

    try:
        track_id = 1
        lat_base = 34.123456
        lon_base = -118.654321

        while True:
            # Simulate moving detections
            for i in range(3):
                # Create mock detection
                detection = Detection(
                    bbox=(100 + i * 50, 100, 200 + i * 50, 200),
                    confidence=0.80 + i * 0.05,
                    class_id=0,
                    class_name=["person", "car", "truck"][i],
                    track_id=track_id + i,
                )

                # Generate GPS position (simulate movement)
                offset = (time.time() % 10) / 10000.0
                lat = lat_base + offset + (i * 0.0001)
                lon = lon_base + offset + (i * 0.0001)

                # Generate and queue CoT message
                cot_xml = generator.generate_detection_cot(
                    detection=detection,
                    lat=lat,
                    lon=lon,
                    alt_msl=100.0 + i * 10,
                )

                tak.send_cot(cot_xml)

            # Get stats
            stats = tak.get_stats()
            print(
                f"\rPublished: {stats['messages_sent']} messages | "
                f"Failed: {stats['messages_failed']} | "
                f"Queue: {stats['queue_size']} | "
                f"Connected: {'✓' if stats['connected'] else '✗'}    ",
                end='',
                flush=True
            )

            # Publish every second
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n\nStopping...")

    finally:
        tak.stop()
        print("TAK interface stopped")


def main():
    """Main demo menu."""
    print("=" * 60)
    print("DroneBrain - TAK Integration Demo")
    print("=" * 60)
    print()
    print("Select a test:")
    print("  1. Test CoT message generation (no server needed)")
    print("  2. Test TAK server connection")
    print("  3. Test continuous publishing")
    print("  q. Quit")
    print()

    choice = input("Enter choice: ").strip()

    if choice == "1":
        test_cot_generation()
    elif choice == "2":
        test_tak_connection()
    elif choice == "3":
        test_continuous_publishing()
    elif choice.lower() == "q":
        print("Exiting...")
        return
    else:
        print("Invalid choice")


if __name__ == "__main__":
    main()
