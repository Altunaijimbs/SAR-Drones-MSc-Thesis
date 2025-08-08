#!/usr/bin/env python3
"""
Quick AirSim Demo - No PX4, No MAVROS, No ROS2!
Just pure AirSim for beautiful smooth flight
"""

import airsim
import time
import sys

def fly_square_pattern(size=20):
    """Fly a square pattern using AirSim native control"""
    print("Connecting to AirSim...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    
    print("Arming drone...")
    client.armDisarm(True)
    
    # Take off
    print("Taking off...")
    client.takeoffAsync().join()
    time.sleep(2)
    
    # Move to starting altitude
    z = -10  # 10 meters up (NED coordinates)
    print(f"Moving to altitude {-z}m...")
    client.moveToZAsync(z, 2).join()
    
    # Define square pattern (NED coordinates)
    print(f"Flying {size}m square pattern at 5 m/s...")
    square_path = [
        airsim.Vector3r(0, 0, z),      # Start
        airsim.Vector3r(size, 0, z),   # Right
        airsim.Vector3r(size, size, z), # Forward-right
        airsim.Vector3r(0, size, z),    # Forward
        airsim.Vector3r(0, 0, z),       # Back to start
    ]
    
    # Fly the pattern with smooth curves!
    result = client.moveOnPathAsync(
        square_path,
        velocity=5,  # 5 m/s
        timeout_sec=60,
        drivetrain=airsim.DrivetrainType.ForwardOnly,
        yaw_mode=airsim.YawMode(False, 0),
        lookahead=7.5,  # Dynamic lookahead for smooth turns
        adaptive_lookahead=1
    ).join()
    
    print("Pattern complete! Hovering...")
    client.hoverAsync().join()
    time.sleep(2)
    
    print("Landing...")
    client.landAsync().join()
    
    print("Disarming...")
    client.armDisarm(False)
    client.enableApiControl(False)
    print("Done!")

def fly_search_pattern():
    """Fly a search and rescue pattern"""
    print("Connecting to AirSim...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    
    print("Arming drone...")
    client.armDisarm(True)
    
    print("Taking off...")
    client.takeoffAsync().join()
    time.sleep(2)
    
    z = -15  # 15 meters altitude
    print(f"Moving to search altitude {-z}m...")
    client.moveToZAsync(z, 2).join()
    
    # Lawn mower search pattern
    print("Starting search pattern...")
    search_path = []
    width = 30
    length = 30
    strips = 5
    strip_width = width / strips
    
    for i in range(strips):
        x = i * strip_width
        if i % 2 == 0:
            # Go forward
            search_path.append(airsim.Vector3r(x, 0, z))
            search_path.append(airsim.Vector3r(x, length, z))
        else:
            # Come back
            search_path.append(airsim.Vector3r(x, length, z))
            search_path.append(airsim.Vector3r(x, 0, z))
    
    # Execute search pattern
    print(f"Executing {strips}-strip search pattern...")
    result = client.moveOnPathAsync(
        search_path,
        velocity=6,  # 6 m/s for search
        timeout_sec=120,
        drivetrain=airsim.DrivetrainType.ForwardOnly,
        yaw_mode=airsim.YawMode(False, 0),
        lookahead=9,  # Higher lookahead for smoother search
        adaptive_lookahead=1
    ).join()
    
    print("Search complete! Returning to home...")
    client.moveToPositionAsync(0, 0, z, 5).join()
    
    print("Landing...")
    client.landAsync().join()
    
    print("Disarming...")
    client.armDisarm(False)
    client.enableApiControl(False)
    print("Search mission complete!")

if __name__ == "__main__":
    print("""
    ╔══════════════════════════════════════════════════════╗
    ║         AIRSIM DIRECT CONTROL DEMO                   ║
    ║                                                       ║
    ║  NO PX4 REQUIRED!                                    ║
    ║  NO MAVROS REQUIRED!                                 ║
    ║  NO ROS2 REQUIRED!                                   ║
    ║                                                       ║
    ║  Just pure, smooth AirSim flight!                    ║
    ╚══════════════════════════════════════════════════════╝
    
    Select pattern:
    1. Square Pattern (20m)
    2. Search and Rescue Pattern
    """)
    
    choice = input("Enter choice (1 or 2): ")
    
    if choice == "1":
        fly_square_pattern(20)
    elif choice == "2":
        fly_search_pattern()
    else:
        print("Invalid choice!")
        sys.exit(1)