#!/usr/bin/env python3
"""
Direct camera test - tries all possible API signatures
"""

import cosysairsim as airsim
import numpy as np
import cv2

print("Testing AirSim camera API...")
print("-" * 60)

# Connect
client = airsim.MultirotorClient()
client.confirmConnection()
print("Connected to AirSim")

# Test 1: simGetImage with 2 arguments (camera_name, image_type)
print("\nTest 1: simGetImage(camera_name, image_type)")
try:
    image = client.simGetImage("0", airsim.ImageType.Scene)
    if image:
        print(f"  ✅ Success! Got {len(image)} bytes")
except Exception as e:
    print(f"  ❌ Failed: {e}")

# Test 2: simGetImage with 3 arguments (camera_name, image_type, vehicle_name)
print("\nTest 2: simGetImage(camera_name, image_type, vehicle_name)")
try:
    image = client.simGetImage("0", airsim.ImageType.Scene, "Drone1")
    if image:
        print(f"  ✅ Success! Got {len(image)} bytes")
except Exception as e:
    print(f"  ❌ Failed: {e}")

# Test 3: simGetImages with ImageRequest (old style)
print("\nTest 3: simGetImages([ImageRequest]) - no vehicle name")
try:
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])
    if responses and responses[0].width > 0:
        print(f"  ✅ Success! Got {responses[0].width}x{responses[0].height} image")
        # Try to save it
        response = responses[0]
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(response.height, response.width, 3)
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite("test_image.png", img_bgr)
        print("  📸 Saved test_image.png")
except Exception as e:
    print(f"  ❌ Failed: {e}")

# Test 4: simGetImages with vehicle_name
print("\nTest 4: simGetImages([ImageRequest], vehicle_name)")
try:
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ], "Drone1")
    if responses and responses[0].width > 0:
        print(f"  ✅ Success! Got {responses[0].width}x{responses[0].height} image")
except Exception as e:
    print(f"  ❌ Failed: {e}")

# Test 5: Different camera names
print("\nTest 5: Testing different camera names...")
camera_names = ["0", "1", "front_center", "front", "bottom"]
for cam in camera_names:
    try:
        responses = client.simGetImages([
            airsim.ImageRequest(cam, airsim.ImageType.Scene, False, False)
        ])
        if responses and responses[0].width > 0:
            print(f"  ✅ Camera '{cam}' works!")
    except:
        pass

print("\n" + "=" * 60)
print("Test complete. Check which methods worked above.")