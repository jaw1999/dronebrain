#!/usr/bin/env python3
"""
Simple gimbal control test.

Tests basic gimbal movement to verify communication is working.
"""

import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.interfaces.siyi_interface import SiyiInterface
from src.core.state import SystemState

def main():
    print("=" * 60)
    print("Gimbal Control Test")
    print("=" * 60)

    # Create state and gimbal interface
    state = SystemState()
    siyi = SiyiInterface(state, ip="192.168.144.25", port=37260)

    # Connect
    print("\nConnecting to gimbal...")
    if not siyi.connect():
        print("ERROR: Failed to connect to gimbal")
        return

    print("✓ Connected successfully")

    # Start telemetry
    siyi.start()
    time.sleep(1)  # Let telemetry update

    # Get current position
    gimbal_state = state.get_gimbal_state()
    print(f"\nCurrent position: Yaw={gimbal_state.yaw:.1f}°, Pitch={gimbal_state.pitch:.1f}°")

    # Test 1: Center
    print("\nTest 1: Centering gimbal...")
    siyi.center()
    time.sleep(2)
    gimbal_state = state.get_gimbal_state()
    print(f"Position: Yaw={gimbal_state.yaw:.1f}°, Pitch={gimbal_state.pitch:.1f}°")

    # Test 2: Move right
    print("\nTest 2: Moving right (yaw=+30°)...")
    siyi.set_attitude(yaw=30.0, skip_rate_limit=True)
    time.sleep(2)
    gimbal_state = state.get_gimbal_state()
    print(f"Position: Yaw={gimbal_state.yaw:.1f}°, Pitch={gimbal_state.pitch:.1f}°")

    # Test 3: Move left
    print("\nTest 3: Moving left (yaw=-30°)...")
    siyi.set_attitude(yaw=-30.0, skip_rate_limit=True)
    time.sleep(2)
    gimbal_state = state.get_gimbal_state()
    print(f"Position: Yaw={gimbal_state.yaw:.1f}°, Pitch={gimbal_state.pitch:.1f}°")

    # Test 4: Tilt down
    print("\nTest 4: Tilting down (pitch=-45°)...")
    siyi.set_attitude(pitch=-45.0, skip_rate_limit=True)
    time.sleep(2)
    gimbal_state = state.get_gimbal_state()
    print(f"Position: Yaw={gimbal_state.yaw:.1f}°, Pitch={gimbal_state.pitch:.1f}°")

    # Test 5: Velocity control
    print("\nTest 5: Testing velocity control (slow pan right)...")
    print("Panning right at 10°/sec for 3 seconds...")
    siyi.reset_velocity_control()
    start_time = time.time()
    while time.time() - start_time < 3.0:
        siyi.set_gimbal_velocity(yaw=10.0, pitch=0.0)
        time.sleep(0.05)  # 20Hz

    gimbal_state = state.get_gimbal_state()
    print(f"Position: Yaw={gimbal_state.yaw:.1f}°, Pitch={gimbal_state.pitch:.1f}°")

    # Return to center
    print("\nReturning to center...")
    siyi.center()
    time.sleep(2)

    # Cleanup
    siyi.stop()
    print("\nTest complete!")

if __name__ == "__main__":
    main()
