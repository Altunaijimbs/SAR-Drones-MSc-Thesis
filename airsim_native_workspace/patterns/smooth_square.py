#!/usr/bin/env python3
"""
Smooth Square Pattern - Uses moveOnPath for perfect corners
Demonstrates the difference from point-to-point navigation
"""

import cosysairsim as airsim
import time
import argparse

def fly_smooth_square(size=20, altitude=15, speed=5):
    print("=" * 60)
    print("       SMOOTH SQUARE PATTERN")
    print("=" * 60)
    print(f"Size: {size}m | Altitude: {altitude}m | Speed: {speed} m/s")
    print()
    
    # Connect
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
    
    # Create square path with extra points for smooth corners
    print(f"Creating {size}m square pattern...")
    
    # Center the square around current position
    half = size / 2
    
    # Main square points
    square_path = [
        airsim.Vector3r(-half, -half, z),  # Bottom-left
        airsim.Vector3r(half, -half, z),   # Bottom-right
        airsim.Vector3r(half, half, z),    # Top-right
        airsim.Vector3r(-half, half, z),   # Top-left
        airsim.Vector3r(-half, -half, z),  # Back to start
    ]
    
    # Calculate total distance
    total_distance = size * 4
    estimated_time = total_distance / speed
    
    print(f"Flying square pattern...")
    print(f"  Total distance: {total_distance}m")
    print(f"  Estimated time: {estimated_time:.1f} seconds")
    print()
    
    # Dynamic lookahead for smooth corners
    lookahead = speed + (speed / 2)  # Same as path.py example
    
    # Execute with smooth corners!
    start_time = time.time()
    
    result = client.moveOnPathAsync(
        path=square_path,
        velocity=speed,
        timeout_sec=estimated_time + 10,
        drivetrain=airsim.DrivetrainType.ForwardOnly,
        yaw_mode=airsim.YawMode(False, 0),  # Let path determine yaw
        lookahead=lookahead,  # Key for smooth turns!
        adaptive_lookahead=1  # Adaptive scaling
    ).join()
    
    elapsed = time.time() - start_time
    
    print(f"✅ Pattern complete in {elapsed:.1f} seconds")
    
    # Hover briefly
    print("Hovering...")
    client.hoverAsync().join()
    time.sleep(2)
    
    # Return to center
    print("Returning to center...")
    client.moveToPositionAsync(0, 0, z, velocity=speed).join()
    
    # Land
    print("Landing...")
    client.landAsync().join()
    
    # Disarm
    print("Disarming...")
    client.armDisarm(False)
    client.enableApiControl(False)
    
    print()
    print("=" * 60)
    print("✅ SMOOTH SQUARE COMPLETE!")
    print("=" * 60)
    print()
    print("Notice how the drone:")
    print("  • Didn't stop at corners")
    print("  • Maintained speed throughout")
    print("  • Flew smooth curves instead of sharp angles")
    print()
    print("This is the power of moveOnPath with lookahead!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Fly a smooth square pattern')
    parser.add_argument('--size', type=float, default=20, help='Square size in meters')
    parser.add_argument('--altitude', type=float, default=15, help='Flight altitude in meters')
    parser.add_argument('--speed', type=float, default=5, help='Flight speed in m/s')
    
    args = parser.parse_args()
    
    try:
        fly_smooth_square(args.size, args.altitude, args.speed)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("\nMake sure UE5 is running with PLAY pressed")