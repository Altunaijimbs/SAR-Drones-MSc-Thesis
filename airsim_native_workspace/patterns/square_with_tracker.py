#!/usr/bin/env python3
"""
Square Pattern with Real-time Tracking
Flies a square pattern while showing live position tracking
"""

import cosysairsim as airsim
import time
import argparse
import subprocess
import sys
import os

def fly_square_with_tracking(size=20, altitude=15, speed=5):
    print("=" * 60)
    print("       SQUARE PATTERN WITH LIVE TRACKING")
    print("=" * 60)
    print(f"Size: {size}m | Altitude: {altitude}m | Speed: {speed} m/s")
    print()
    
    # Start the tracker in a separate process
    tracker_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'visualizer', 'drone_tracker.py'
    )
    
    print("Starting position tracker...")
    tracker_process = subprocess.Popen([sys.executable, tracker_path])
    time.sleep(2)  # Give tracker time to start
    
    try:
        # Connect to AirSim
        print("Connecting to AirSim...")
        client = airsim.MultirotorClient()
        client.confirmConnection()
        client.enableApiControl(True)
        
        # Arm and takeoff
        print("Arming...")
        client.armDisarm(True)
        
        state = client.getMultirotorState()
        if state.landed_state == airsim.LandedState.Landed:
            print("Taking off...")
            client.takeoffAsync().join()
        
        # Move to altitude
        z = -altitude  # NED coordinates
        print(f"Moving to {altitude}m altitude...")
        client.moveToZAsync(z, velocity=3).join()
        
        print(f"\n📍 Flying {size}m square pattern...")
        print("Watch the tracker window to see the drone's path!\n")
        
        # Get current position as starting point
        pose = client.simGetVehiclePose()
        start_x = pose.position.x_val
        start_y = pose.position.y_val
        
        # Create square waypoints
        half = size / 2
        waypoints = [
            (start_x - half, start_y - half),  # Bottom-left
            (start_x + half, start_y - half),  # Bottom-right  
            (start_x + half, start_y + half),  # Top-right
            (start_x - half, start_y + half),  # Top-left
            (start_x - half, start_y - half),  # Back to start
        ]
        
        # Fly the square
        for i, (x, y) in enumerate(waypoints):
            corner_names = ["Bottom-left", "Bottom-right", "Top-right", "Top-left", "Start"]
            print(f"Flying to {corner_names[i]} corner: ({x:.1f}, {y:.1f})")
            
            # Use moveToPositionAsync for point-to-point movement
            client.moveToPositionAsync(x, y, z, velocity=speed).join()
            
            # Pause briefly at each corner
            time.sleep(0.5)
        
        print("\n✅ Pattern complete!")
        
        # Hover for a bit to show completed path
        print("Hovering for 5 seconds...")
        time.sleep(5)
        
        # Land
        print("Landing...")
        client.landAsync().join()
        
        # Disarm
        print("Disarming...")
        client.armDisarm(False)
        client.enableApiControl(False)
        
        print("\n" + "="*60)
        print("FLIGHT COMPLETE!")
        print("="*60)
        print("Check the tracker window to see the full flight path")
        print("Close the tracker window when done")
        
    except KeyboardInterrupt:
        print("\n⚠️  Flight interrupted by user")
        print("Emergency landing...")
        client.landAsync().join()
        client.armDisarm(False)
        client.enableApiControl(False)
        
    except Exception as e:
        print(f"❌ Error: {e}")
        print("Attempting emergency landing...")
        try:
            client.landAsync().join()
            client.armDisarm(False)
            client.enableApiControl(False)
        except:
            pass
        
    finally:
        # Keep tracker running for review
        print("\nPress Ctrl+C to close the tracker...")
        try:
            tracker_process.wait()
        except KeyboardInterrupt:
            tracker_process.terminate()
            print("Tracker closed")

def main():
    parser = argparse.ArgumentParser(description='Square pattern with live tracking')
    parser.add_argument('--size', type=float, default=20,
                       help='Size of square in meters (default: 20)')
    parser.add_argument('--altitude', type=float, default=15,
                       help='Flight altitude in meters (default: 15)')
    parser.add_argument('--speed', type=float, default=5,
                       help='Flight speed in m/s (default: 5)')
    args = parser.parse_args()
    
    fly_square_with_tracking(args.size, args.altitude, args.speed)

if __name__ == "__main__":
    main()