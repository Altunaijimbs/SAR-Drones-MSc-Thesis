#!/usr/bin/env python3
"""
Simple Hover Test - Basic flight test
Shows arm, takeoff, hover, land sequence
"""

import cosysairsim as airsim
import time

def hover_test():
    print("=" * 60)
    print("       AIRSIM HOVER TEST")
    print("=" * 60)
    print()
    
    # Connect
    print("Connecting to AirSim...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("✅ Connected")
    
    # Enable API control
    print("Enabling API control...")
    client.enableApiControl(True)
    print("✅ API control enabled")
    
    # Arm
    print("Arming drone...")
    client.armDisarm(True)
    print("✅ Armed")
    
    # Takeoff
    print("Taking off...")
    client.takeoffAsync().join()
    print("✅ Takeoff complete")
    
    # Get altitude
    state = client.getMultirotorState()
    altitude = -state.kinematics_estimated.position.z_val
    print(f"Current altitude: {altitude:.1f} m")
    
    # Move to 10m altitude
    print("Moving to 10m altitude...")
    client.moveToZAsync(-10, velocity=2).join()
    print("✅ At 10m")
    
    # Hover
    print("Hovering for 5 seconds...")
    client.hoverAsync().join()
    
    for i in range(5, 0, -1):
        print(f"  {i}...")
        time.sleep(1)
    
    print("✅ Hover complete")
    
    # Small movement
    print("Moving forward 5 meters...")
    client.moveByVelocityAsync(2, 0, 0, 2.5).join()
    print("✅ Movement complete")
    
    # Return
    print("Returning to start position...")
    client.moveToPositionAsync(0, 0, -10, velocity=3).join()
    print("✅ Back at start")
    
    # Land
    print("Landing...")
    client.landAsync().join()
    print("✅ Landed")
    
    # Disarm
    print("Disarming...")
    client.armDisarm(False)
    client.enableApiControl(False)
    print("✅ Disarmed")
    
    print()
    print("=" * 60)
    print("✅ HOVER TEST COMPLETE!")
    print("=" * 60)

if __name__ == "__main__":
    try:
        hover_test()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("\nMake sure:")
        print("1. UE5 is running with PLAY pressed")
        print("2. Run test_connection.py first")