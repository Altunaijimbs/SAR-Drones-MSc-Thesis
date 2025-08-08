#!/usr/bin/env python3
"""
Test if we can use Colosseum (Microsoft's official AirSim successor)
or find another way to get camera working
"""

import sys
import subprocess

print("Checking for alternative AirSim clients...")
print("=" * 60)

# Check what's available
packages_to_try = [
    ("msgpack-rpc-python", "Basic RPC client"),
    ("airsim", "Standard AirSim"),
    ("colosseum", "Microsoft Colosseum (AirSim successor)"),
]

print("\n1. Checking installed packages:")
for pkg, desc in packages_to_try:
    try:
        __import__(pkg.replace("-", "_"))
        print(f"  ✅ {pkg}: {desc}")
    except ImportError:
        print(f"  ❌ {pkg}: Not installed")

print("\n2. Attempting to install Colosseum (AirSim's official successor)...")
print("-" * 40)

# Try to install Colosseum
result = subprocess.run(
    ["pip3", "install", "cosysairsim", "--force-reinstall", "--no-deps"],
    capture_output=True,
    text=True
)

if result.returncode == 0:
    print("✅ Installed cosysairsim without dependencies")
    
    # Now let's test it
    print("\n3. Testing cosysairsim camera...")
    print("-" * 40)
    
    test_code = """
import cosysairsim as airsim
import numpy as np

client = airsim.MultirotorClient()
client.confirmConnection()
print("Connected with cosysairsim!")

# Try to get camera image
try:
    # Method 1: Standard call
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])
    if responses and responses[0].width > 0:
        print(f"✅ Camera works! Got {responses[0].width}x{responses[0].height} image")
    else:
        print("❌ No image data")
except Exception as e:
    print(f"❌ Camera failed: {e}")
"""
    
    with open("test_cosys_camera.py", "w") as f:
        f.write(test_code)
    
    # Run the test
    result = subprocess.run(["python3", "test_cosys_camera.py"], capture_output=True, text=True)
    print(result.stdout)
    if result.stderr:
        print("Errors:", result.stderr)
else:
    print("❌ Failed to install cosysairsim")
    print(result.stderr)

print("\n" + "=" * 60)
print("ALTERNATIVE SOLUTION: Use PX4 mode instead!")
print("=" * 60)
print("""
The PX4 mode has a working camera system through ROS2!

To switch to PX4 mode:
1. cd ~/SAR-Drones-MSc-Thesis/airsim_native_workspace
2. ./switch_mode.sh  (choose option 1 for PX4)
3. Restart UE5
4. Start PX4: cd ~/PX4-Autopilot && make px4_sitl_default none_iris
5. Use the ROS2 camera topic: /airsim_node/PX4/front_center_Scene/image

The PX4 system already has:
- Working camera feed via ROS2
- YOLO integration in drone_vision_interpreter
- Web interface at port 5000
- All the infrastructure for vision-based SAR

Would you like to switch to PX4 mode for YOLO integration?
""")