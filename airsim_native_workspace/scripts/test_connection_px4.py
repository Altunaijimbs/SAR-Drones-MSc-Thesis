#!/usr/bin/env python3
"""
Test Connection for PX4 Configuration
Works with your current settings.json
"""

import cosysairsim as airsim
import time
import sys

def test_px4_connection():
    print("=" * 60)
    print("       AIRSIM PX4 CONNECTION TEST")
    print("=" * 60)
    print()
    print("Your configuration:")
    print("  Vehicle: PX4 (PX4Multirotor)")
    print("  Camera: front_center")
    print("  Mode: PX4 with LockStep")
    print()
    
    print("Connecting to AirSim...")
    
    try:
        # Connect to AirSim
        client = airsim.MultirotorClient()
        client.confirmConnection()
        print("✅ Connected to AirSim!")
        print()
        
        # For PX4 mode, we need to specify the vehicle name
        vehicle_name = "PX4"
        camera_name = "front_center"
        
        # Get drone state
        print("Getting PX4 drone state...")
        state = client.getMultirotorState(vehicle_name=vehicle_name)
        
        pos = state.kinematics_estimated.position
        print(f"Position (NED):")
        print(f"  X (North): {pos.x_val:.2f} m")
        print(f"  Y (East):  {pos.y_val:.2f} m")
        print(f"  Z (Down):  {pos.z_val:.2f} m")
        print()
        
        # Test camera with correct name and vehicle
        print("Testing camera 'front_center'...")
        try:
            # Try with vehicle name
            camera_info = client.simGetCameraInfo(camera_name, vehicle_name, False)
            print(f"✅ Camera info retrieved!")
            print(f"   FOV: {camera_info.fov} degrees")
        except Exception as e:
            print(f"⚠️  Camera info error: {e}")
            print("   But images should still work!")
        
        # Test getting images
        print()
        print("Testing image capture...")
        responses = client.simGetImages([
            airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)
        ], vehicle_name=vehicle_name)
        
        if responses and len(responses) > 0:
            response = responses[0]
            print(f"✅ Got image! Size: {response.width}x{response.height}")
        else:
            print("❌ No image received")
        
        # Test API control
        print()
        print("Testing API control...")
        client.enableApiControl(True, vehicle_name)
        print("✅ API control enabled")
        
        # Note about PX4 mode
        print()
        print("=" * 60)
        print("⚠️  IMPORTANT: PX4 MODE NOTES")
        print("=" * 60)
        print()
        print("You're in PX4 mode, which means:")
        print("1. PX4 SITL must be running for flight")
        print("2. Use vehicle_name='PX4' in all API calls")
        print("3. Camera is 'front_center' not '0'")
        print()
        print("For pure AirSim (no PX4), see Option 2 below")
        
        return True
        
    except Exception as e:
        print(f"\n❌ Connection failed: {e}")
        return False

if __name__ == "__main__":
    success = test_px4_connection()
    
    if success:
        print("\n" + "=" * 60)
        print("✅ PX4 MODE CONNECTION SUCCESSFUL!")
        print("=" * 60)
        print()
        print("Your camera is: 'front_center'")
        print("Your vehicle is: 'PX4'")
        print()
        print("Update your scripts to use:")
        print('  camera_name = "front_center"')
        print('  vehicle_name = "PX4"')
    
    sys.exit(0 if success else 1)