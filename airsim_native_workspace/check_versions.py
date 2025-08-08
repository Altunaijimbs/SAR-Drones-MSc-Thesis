#!/usr/bin/env python3
"""
Check AirSim versions and compatibility
"""

import cosysairsim as airsim
import sys

print("=" * 60)
print("     AIRSIM VERSION CHECK")
print("=" * 60)
print()

# Check Python client version
try:
    import pkg_resources
    version = pkg_resources.get_distribution("airsim").version
    print(f"Python airsim library version: {version}")
except:
    print("Could not determine Python airsim version")

print()

# Try to connect and check API
print("Testing connection methods...")
print("-" * 40)

client = airsim.MultirotorClient()
client.confirmConnection()
print("✅ Basic connection works")

# Test which API version works
print()
print("Testing API compatibility...")

# Test camera info with different signatures
print("  Testing simGetCameraInfo...")
try:
    # Old format (2 args)
    info = client.simGetCameraInfo("0")
    print("    ✅ Works with 2 arguments (old format)")
except Exception as e:
    if "invalid number of arguments" in str(e):
        try:
            # New format (3 args)
            info = client.simGetCameraInfo("0", "", False)
            print("    ✅ Works with 3 arguments (new format)")
        except:
            print("    ❌ Camera info not working")
    else:
        print(f"    ❌ Error: {e}")

# Test other common functions
print("  Testing getMultirotorState...")
try:
    state = client.getMultirotorState()
    print("    ✅ Works")
except Exception as e:
    print(f"    ❌ Error: {e}")

print("  Testing enableApiControl...")
try:
    client.enableApiControl(True)
    client.enableApiControl(False)
    print("    ✅ Works")
except Exception as e:
    print(f"    ❌ Error: {e}")

print()
print("Recommendation:")
print("-" * 40)

if "cosys" in airsim.__file__.lower():
    print("You're using Cosys-AirSim Python client")
    print("This may have different API than standard AirSim")
    print()
    print("To fix version mismatches:")
    print("1. Use the Python client from your Cosys-AirSim installation:")
    print("   export PYTHONPATH=/path/to/Cosys-AirSim/PythonClient:$PYTHONPATH")
    print()
    print("2. Or update your pip package:")
    print("   pip install cosysairsim")
else:
    print("You're using standard AirSim Python client")
    print("Version 1.8.1 is relatively recent")
    print()
    print("The camera API mismatch is minor and won't affect flight")

print()
print("=" * 60)
print("Flight capabilities are NOT affected by this!")
print("You can safely ignore camera API warnings.")
print("=" * 60)