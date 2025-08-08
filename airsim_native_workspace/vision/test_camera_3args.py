#!/usr/bin/env python3
"""
Test simGetImages with 3 arguments - figure out what the third arg is
"""

import cosysairsim as airsim
import numpy as np
import cv2

print("Testing simGetImages with 3 arguments...")
print("-" * 60)

# Connect
client = airsim.MultirotorClient()
client.confirmConnection()
print("Connected to AirSim\n")

# The error says it expects 3 arguments for simGetImages
# Standard signature is: simGetImages(requests, vehicle_name, external)
# Let's test different combinations

print("Test 1: simGetImages(requests, vehicle_name, external=False)")
try:
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ], "Drone1", False)
    
    if responses and len(responses) > 0:
        response = responses[0]
        if response.width > 0:
            print(f"  ✅ SUCCESS! Got {response.width}x{response.height} image")
            
            # Save the image
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(response.height, response.width, 3)
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            cv2.imwrite("camera_works.png", img_bgr)
            print("  📸 Saved camera_works.png")
            print("\n  Camera is working! Use this method in your scripts.")
        else:
            print(f"  ❌ Got response but no image data")
    else:
        print(f"  ❌ No response")
except Exception as e:
    print(f"  ❌ Failed: {e}")

print("\nTest 2: simGetImages(requests, vehicle_name='', external=False)")
try:
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ], "", False)
    
    if responses and responses[0].width > 0:
        print(f"  ✅ SUCCESS with empty vehicle name!")
except Exception as e:
    print(f"  ❌ Failed: {e}")

print("\nTest 3: Testing with external=True")
try:
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ], "Drone1", True)
    
    if responses and responses[0].width > 0:
        print(f"  ✅ Works with external=True")
except Exception as e:
    print(f"  ❌ Failed: {e}")

print("\n" + "=" * 60)