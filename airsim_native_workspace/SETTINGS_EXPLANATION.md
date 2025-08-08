# Your AirSim Settings Explained

## Current Configuration (PX4 Mode)

Your `settings.json` is configured for **PX4 SITL mode**, not pure AirSim SimpleFlight!

### Key Points:
- **Vehicle Type**: `PX4Multirotor` (requires PX4 SITL)
- **Vehicle Name**: `"PX4"` (not "Drone1")
- **Camera Name**: `"front_center"` (not "0")
- **LockStep**: Enabled (synchronizes with PX4)

## You Have Two Choices:

### Choice 1: Keep PX4 Mode (Your Current Setup)
This is what you've been using with `launch_hybrid_system.sh`

**Pros:**
- Realistic autopilot simulation
- Can deploy to real drone
- Full PX4 features

**Cons:**
- Requires PX4 SITL running
- More complex
- Can't use simple AirSim native commands

**To use this mode:**
```python
# Must specify vehicle and camera names
client = airsim.MultirotorClient()
vehicle_name = "PX4"
camera_name = "front_center"

# Get images
responses = client.simGetImages([
    airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)
], vehicle_name="PX4")

# Control requires PX4 running
# Cannot use moveOnPathAsync directly!
```

### Choice 2: Switch to SimpleFlight (Pure AirSim)
For smooth patterns without PX4

**Pros:**
- No PX4 needed
- Direct control with moveOnPathAsync
- Simpler and smoother
- Perfect for demos

**Cons:**
- Simulation only
- Can't deploy to real drone

**To switch:**
1. Backup current settings:
   ```bash
   cp ~/Documents/AirSim/settings.json ~/Documents/AirSim/settings_px4.json
   ```

2. Copy SimpleFlight settings:
   ```bash
   cp settings_simpleflight.json ~/Documents/AirSim/settings.json
   ```

3. Restart UE5

## Quick Test for Your Current PX4 Mode:

```bash
# Test with PX4 settings
python3 scripts/test_connection_px4.py
```

## Code Changes Needed for PX4 Mode:

### Camera Access:
```python
# Wrong (for SimpleFlight)
camera_name = "0"
vehicle_name = ""

# Correct (for your PX4 setup)
camera_name = "front_center"
vehicle_name = "PX4"
```

### Getting Images:
```python
# For PX4 mode
responses = client.simGetImages([
    airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)
], vehicle_name="PX4")
```

### Movement (PX4 Mode):
```python
# PX4 mode CANNOT use moveOnPathAsync directly!
# You need PX4 running and must use:
# - MAVROS commands
# - Or switch to SimpleFlight
```

## Recommendation:

For your thesis demo with smooth patterns:
1. **Use SimpleFlight** for demo videos (smooth, simple)
2. **Use PX4 mode** to show real-world system

You can have both! Just swap settings.json files:
- `settings_px4.json` - Your current PX4 setup
- `settings_simpleflight.json` - For native AirSim demos