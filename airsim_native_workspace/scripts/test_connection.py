#!/usr/bin/env python3
"""
Test AirSim Connection - First script to run!
No PX4, No MAVROS, just AirSim
"""

import cosysairsim as airsim
import time
import sys

def test_connection():
    print("=" * 60)
    print("       AIRSIM NATIVE CONNECTION TEST")
    print("=" * 60)
    print()
    
    print("Prerequisites:")
    print("  ✓ Unreal Engine 5 running")
    print("  ✓ AirSim plugin loaded")
    print("  ✓ PLAY button pressed in UE5")
    print()
    
    print("Attempting to connect to AirSim...")
    
    try:
        # Create client
        client = airsim.MultirotorClient()
        
        # Try to connect
        client.confirmConnection()
        print("✅ Successfully connected to AirSim!")
        print()
        
        # Get drone state
        print("Drone State Information:")
        print("-" * 40)
        
        state = client.getMultirotorState()
        
        # Position
        pos = state.kinematics_estimated.position
        print(f"Position (NED):")
        print(f"  X (North): {pos.x_val:.2f} m")
        print(f"  Y (East):  {pos.y_val:.2f} m")
        print(f"  Z (Down):  {pos.z_val:.2f} m")
        print()
        
        # Orientation
        orientation = state.kinematics_estimated.orientation
        print(f"Orientation (Quaternion):")
        print(f"  w: {orientation.w_val:.3f}")
        print(f"  x: {orientation.x_val:.3f}")
        print(f"  y: {orientation.y_val:.3f}")
        print(f"  z: {orientation.z_val:.3f}")
        print()
        
        # Status
        print(f"Status:")
        print(f"  Landed: {state.landed_state}")
        print(f"  Collision: {state.collision.has_collided}")
        print(f"  API Control enabled: {client.isApiControlEnabled()}")
        print()
        
        # Test API control
        print("Testing API control...")
        client.enableApiControl(True)
        print("✅ API control enabled successfully!")
        
        # Get camera info (with version compatibility)
        print()
        print("Camera Information:")
        print("-" * 40)
        try:
            # Try newer API format (3 arguments - camera_name, vehicle_name, external)
            camera_info = client.simGetCameraInfo("0", "", False)
            print(f"  FOV: {camera_info.fov} degrees")
            print(f"  Position: ({camera_info.pose.position.x_val:.2f}, "
                  f"{camera_info.pose.position.y_val:.2f}, "
                  f"{camera_info.pose.position.z_val:.2f})")
        except:
            # Fall back to older format or skip
            print("  Camera info not available (API version mismatch)")
            print("  This is OK - flight will still work!")
        
        # Disable API control
        client.enableApiControl(False)
        
        print()
        print("=" * 60)
        print("✅ CONNECTION TEST SUCCESSFUL!")
        print("=" * 60)
        print()
        print("You can now run flight scripts!")
        print("Try: python3 scripts/hover_test.py")
        
        return True
        
    except Exception as e:
        print()
        print("❌ CONNECTION FAILED!")
        print("-" * 40)
        print(f"Error: {e}")
        print()
        print("Troubleshooting:")
        print("1. Is Unreal Engine running?")
        print("2. Did you press PLAY in UE5?")
        print("3. Is AirSim plugin enabled?")
        print("4. Check ~/Documents/AirSim/settings.json")
        print()
        return False

if __name__ == "__main__":
    success = test_connection()
    sys.exit(0 if success else 1)