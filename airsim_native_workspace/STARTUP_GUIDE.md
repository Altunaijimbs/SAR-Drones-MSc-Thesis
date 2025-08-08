# AirSim Native System - Complete Startup Guide

## System Architecture
```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   Python    │────▶│  AirSim API  │────▶│  UE5/AirSim │
│   Scripts   │     │   (Direct)   │     │  Simulation │
└─────────────┘     └──────────────┘     └─────────────┘
       ↑                                          │
       │                                          ▼
┌─────────────┐                          ┌─────────────┐
│  LLM/YOLO   │                          │   Camera    │
│  (Optional) │◀─────────────────────────│    Feed     │
└─────────────┘                          └─────────────┘
```

## ⚡ Quick Start (NO PX4/MAVROS!)

### Step 1: Start Unreal Engine with AirSim
```bash
# Navigate to your UE5 project
cd ~/Documents/Unreal\ Projects/Abandoned_Building_Zen_AirSim/

# Start Unreal Editor (if not already running)
# Open the project in UE5 Editor
# Press PLAY button in UE5
```

### Step 2: Verify AirSim Settings
Check `~/Documents/AirSim/settings.json`:
```json
{
  "SimMode": "Multirotor",
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "AutoCreate": true,
      "X": 0, "Y": 0, "Z": 0
    }
  }
}
```

### Step 3: Test Basic Connection
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/airsim_native_workspace
python3 scripts/test_connection.py
```

## 📁 Workspace Structure

```
airsim_native_workspace/
├── scripts/           # Basic control scripts
│   ├── test_connection.py
│   ├── basic_control.py
│   └── emergency_stop.py
├── patterns/          # Pattern flight scripts
│   ├── square_pattern.py
│   ├── search_pattern.py
│   └── spiral_pattern.py
├── llm_integration/   # Natural language control
│   ├── llm_commander.py
│   └── command_parser.py
├── vision/            # YOLO/detection integration
│   ├── camera_feed.py
│   └── yolo_detector.py
└── docs/              # Documentation
```

## 🚀 Complete System Startup (With All Features)

### 1. MINIMAL (Just Flight)
```bash
# Only need UE5 running with AirSim
cd /home/mbs/SAR-Drones-MSc-Thesis/airsim_native_workspace
python3 patterns/search_pattern.py
```

### 2. WITH ROS2 INTEGRATION (For existing nodes)
```bash
# Terminal 1: ROS2 bridge for existing tools
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
ros2 run airsim_native airsim_ros_bridge

# Terminal 2: Pattern generator (if you want to use existing patterns)
ros2 run search_patterns optimized_pattern_generator

# Terminal 3: Native executor
python3 scripts/ros_integrated_executor.py
```

### 3. WITH LLM (Natural Language)
```bash
# Terminal 1: LLM processor
python3 llm_integration/llm_commander.py

# Terminal 2: Command interface
python3 llm_integration/voice_commander.py
# Or text interface:
python3 llm_integration/text_commander.py
```

### 4. WITH VISION (YOLO Detection)
```bash
# Terminal 1: Camera feed from AirSim
python3 vision/camera_feed.py

# Terminal 2: YOLO detector
python3 vision/yolo_detector.py

# Terminal 3: Search with auto-stop on detection
python3 patterns/smart_search.py
```

## 🎮 Control Commands

### Basic Python Control
```python
import airsim

# Connect
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

# Arm and takeoff
client.armDisarm(True)
client.takeoffAsync().join()

# Move to position
client.moveToPositionAsync(x=10, y=10, z=-10, velocity=5).join()

# Smooth path
path = [
    airsim.Vector3r(0, 0, -10),
    airsim.Vector3r(20, 0, -10),
    airsim.Vector3r(20, 20, -10)
]
client.moveOnPathAsync(path, velocity=5).join()

# Land
client.landAsync().join()
client.armDisarm(False)
```

## ⚠️ Key Differences from PX4/MAVROS System

| Feature | PX4/MAVROS | AirSim Native |
|---------|------------|---------------|
| **Startup** | PX4 → MAVROS → ROS2 | Just UE5 |
| **Arming** | ROS2 service call | `client.armDisarm(True)` |
| **Takeoff** | OFFBOARD mode + commands | `client.takeoffAsync()` |
| **Movement** | Velocity commands via MAVROS | Direct position/path API |
| **Coordinates** | ENU (East-North-Up) | NED (North-East-Down) |
| **Altitude** | Positive up | Negative up (Z = -10 for 10m) |

## 📊 Coordinate System

**IMPORTANT: AirSim uses NED (North-East-Down)**
```
Our System (ENU)          AirSim (NED)
X = East        →         Y = East
Y = North       →         X = North  
Z = Up          →         -Z = Up

Conversion:
AirSim_X = Our_Y
AirSim_Y = Our_X
AirSim_Z = -Our_Z
```

## 🔧 Troubleshooting

### "Connection refused"
- Make sure UE5 is running and PLAY is pressed
- Check AirSim plugin is enabled in UE5

### "Vehicle not ready"
- Wait a few seconds after pressing PLAY
- The drone needs time to spawn

### "Drone not moving"
- Make sure you called `enableApiControl(True)`
- Check you armed with `armDisarm(True)`

### Camera feed issues
- Verify camera names in settings.json
- Default is usually "0" or "front_center"

## 🎯 Quick Test Scripts

### 1. Test Connection
```bash
cat > scripts/test_connection.py << 'EOF'
import airsim
import time

print("Connecting to AirSim...")
client = airsim.MultirotorClient()
client.confirmConnection()
print("Connected!")

state = client.getMultirotorState()
print(f"Drone position: {state.kinematics_estimated.position}")
print(f"Landed state: {state.landed_state}")
EOF

python3 scripts/test_connection.py
```

### 2. Simple Hover Test
```bash
cat > scripts/hover_test.py << 'EOF'
import airsim
import time

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

print("Arming...")
client.armDisarm(True)

print("Taking off...")
client.takeoffAsync().join()

print("Hovering for 5 seconds...")
time.sleep(5)

print("Landing...")
client.landAsync().join()

client.armDisarm(False)
client.enableApiControl(False)
print("Done!")
EOF

python3 scripts/hover_test.py
```

## 🚁 Advantages of Native AirSim

1. **Simplicity**: No complex middleware stack
2. **Performance**: Direct API calls, no message passing overhead
3. **Smoothness**: Native path following with lookahead
4. **Reliability**: Fewer components that can fail
5. **Development Speed**: Test ideas instantly

## 📝 Next Steps

1. Test basic connection first
2. Try the hover test
3. Experiment with path following
4. Add LLM/Vision as needed
5. Build your demo!

Remember: This is for SIMULATION ONLY. For real drones, you need the PX4/MAVROS system!