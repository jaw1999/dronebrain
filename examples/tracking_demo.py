#!/usr/bin/env python3
"""
Autonomous Tracking Demo

Demonstrates how to use the autonomous tracking feature to lock onto
and follow detected targets with the gimbal.

Usage:
    python examples/tracking_demo.py

Then use keyboard commands:
    - Enter track ID to start tracking (e.g., "1", "2", etc.)
    - 's' to stop tracking
    - 'q' to quit
"""

import time
import sys
import os
import logging
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.pipeline_manager import PipelineManager


def setup_logging(log_level: str = "INFO"):
    """
    Configure logging system.

    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR)
    """
    # Create logs directory if it doesn't exist
    log_dir = Path("logs")
    log_dir.mkdir(exist_ok=True)

    # Configure logging format
    log_format = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"

    # Configure root logger
    logging.basicConfig(
        level=getattr(logging, log_level.upper()),
        format=log_format,
        handlers=[
            # Console handler
            logging.StreamHandler(sys.stdout),
            # File handler
            logging.FileHandler(
                log_dir / "tracking_demo.log",
                mode='a',
            ),
        ],
    )

    # Reduce verbosity of some noisy libraries
    logging.getLogger("urllib3").setLevel(logging.WARNING)
    logging.getLogger("PIL").setLevel(logging.WARNING)


def main():
    """Run tracking demo."""
    # Setup logging first
    setup_logging("INFO")

    print("=" * 60)
    print("DroneBrain - Autonomous Tracking Demo")
    print("=" * 60)
    print()

    # Initialize pipeline
    pipeline = PipelineManager(config_path="config/config.yaml")

    if not pipeline.initialize():
        print("ERROR: Failed to initialize pipeline")
        return

    # Start pipeline
    pipeline.start()

    print("\nTracking Controls:")
    print("  - Enter track ID number to start tracking (e.g., '1')")
    print("  - 's' to stop tracking")
    print("  - 'q' to quit")
    print()

    try:
        while True:
            # Print current tracking status
            status = pipeline.get_tracking_status()
            if status['enabled'] and status['target']:
                target = status['target']
                print(
                    f"\rTracking: ID {target['track_id']} ({target['class_name']}) "
                    f"- State: {status['state']} - Frames: {target['frames_tracked']}   ",
                    end='',
                    flush=True
                )
            else:
                print(f"\rTracking: IDLE - No active target                    ", end='', flush=True)

            # Check for user input (non-blocking would be better, but keeping it simple)
            # Note: This is a blocking input, so status updates only happen after input
            # For production, use a proper input handler with select() or threading

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nShutting down...")

    finally:
        pipeline.shutdown()
        print("Demo complete")


def interactive_mode():
    """Interactive mode with command input."""
    # Setup logging first
    setup_logging("INFO")

    print("=" * 60)
    print("DroneBrain - Autonomous Tracking Demo (Interactive)")
    print("=" * 60)
    print()

    # Initialize pipeline
    pipeline = PipelineManager(config_path="config/config.yaml")

    if not pipeline.initialize():
        print("ERROR: Failed to initialize pipeline")
        return

    # Start pipeline
    pipeline.start()

    # Center the gimbal on startup
    print("\nCentering gimbal...")
    if pipeline.siyi:
        pipeline.siyi.center()
        import time
        time.sleep(2)  # Wait for gimbal to center
        print("✓ Gimbal centered")

    print("\nCommands:")
    print("  track <id>  - Start tracking target with ID")
    print("  stop        - Stop tracking")
    print("  status      - Show tracking status")
    print("  center      - Center gimbal")
    print("  quit        - Exit demo")
    print()

    try:
        while True:
            cmd = input("> ").strip().lower()

            if cmd == 'quit' or cmd == 'q':
                break

            elif cmd == 'stop' or cmd == 's':
                pipeline.stop_tracking()
                print("Tracking stopped")

            elif cmd == 'center' or cmd == 'c':
                if pipeline.siyi:
                    pipeline.siyi.center()
                    print("Gimbal centered")
                else:
                    print("Gimbal not available")

            elif cmd == 'status':
                status = pipeline.get_tracking_status()
                print(f"Enabled: {status['enabled']}")
                print(f"State: {status['state']}")
                if status['target']:
                    print(f"Target: {status['target']}")

            elif cmd.startswith('track '):
                try:
                    track_id = int(cmd.split()[1])
                    if pipeline.start_tracking(track_id):
                        print(f"Started tracking: Track ID {track_id}")
                    else:
                        print(f"Failed to start tracking")
                except (ValueError, IndexError):
                    print("Usage: track <id>")

            elif cmd.isdigit():
                track_id = int(cmd)
                if pipeline.start_tracking(track_id):
                    print(f"Started tracking: Track ID {track_id}")
                else:
                    print(f"Failed to start tracking")

            else:
                print("Unknown command. Type 'quit' to exit.")

    except KeyboardInterrupt:
        print("\n\nShutting down...")

    finally:
        pipeline.shutdown()
        print("Demo complete")


if __name__ == "__main__":
    # Run interactive mode
    interactive_mode()
