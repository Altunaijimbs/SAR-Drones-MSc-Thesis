# Fixed Issues Guide

## Issues Identified and Solutions

### 1. Keep-Alive Node Interference ✅ FIXED

**Problem**: Keep-alive node publishes directly to MAVROS topic, overriding search commands after 3 seconds.

**Solution**: 
- Created `smart_keep_alive_node.py` that publishes to `/keepalive/velocity_command`
- Created `velocity_coordinator.py` that manages priorities:
  - Priority 0: Keep-alive (lowest)
  - Priority 1: LLM/manual commands
  - Priority 2: Obstacle avoidance
  - Priority 3: Search patterns (high)
  - Priority 99: Emergency stop (highest)

### 2. QoS Mismatch ✅ FIXED

**Problem**: Grid search can't receive position data due to QoS incompatibility.

**Solution**: 
- Created `fixed_grid_search.py` with proper QoS settings:
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

### 3. LiDAR Not Publishing ⚠️ CONFIGURATION NEEDED

**Problem**: LiDAR topic not being published by AirSim.

**Possible Causes**:
1. LiDAR might be disabled in simulation
2. Topic name might be different

**To Check**:
```bash
# List all AirSim topics
ros2 topic list | grep airsim

# Check if LiDAR is in settings.json
cat ~/Documents/AirSim/settings.json | grep -A 10 LidarSensor1
```

## Quick Test Procedure

### 1. Build Updated Packages
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
colcon build --packages-select llm_controller search_patterns
source install/setup.bash
```

### 2. Use Smart Keep-Alive Instead
```bash
# Terminal 5 - Replace old keep_alive_node with:
ros2 run llm_controller smart_keep_alive_node
```

### 3. Launch Velocity Coordinator
```bash
# Terminal 9 - Add this BEFORE launching search patterns:
ros2 run search_patterns velocity_coordinator
```

### 4. Test Fixed Grid Search
```bash
# Terminal 10 - Launch the fixed grid search:
ros2 run search_patterns fixed_grid_search

# Terminal 11 - Send search command:
ros2 topic pub /search_command std_msgs/msg/String "data: 'start grid search'" --once
```

### 5. Monitor Active Velocity Source
```bash
# Terminal 12 - See which controller is active:
ros2 topic echo /velocity_coordinator/active_source
```

## Updated Launch Sequence Summary

1. **Unreal Engine** → Play
2. **PX4 SITL** → `make px4_sitl none_iris`
3. **AirSim Wrapper** → `ros2 launch airsim_ros_pkgs airsim_node.launch.py`
4. **MAVROS** → `ros2 run mavros mavros_node --ros-args...`
5. **Smart Keep-Alive** → `ros2 run llm_controller smart_keep_alive_node` ✨NEW
6. **Velocity Coordinator** → `ros2 run search_patterns velocity_coordinator` ✨NEW
7. **Arm & Takeoff** → Standard procedure
8. **Vision System** → Standard launch
9. **LLM Controller** → Standard launch
10. **Fixed Grid Search** → `ros2 run search_patterns fixed_grid_search` ✨NEW

## Testing Search Without Interference

```bash
# Simple test - should work now!
ros2 topic pub /search_command std_msgs/msg/String "data: 'search area'" --once

# Monitor to ensure search is controlling:
ros2 topic echo /velocity_coordinator/active_source
# Should show: "Active: search"
```

## Debugging Commands

```bash
# Check all velocity topics
ros2 topic hz /keepalive/velocity_command
ros2 topic hz /search_pattern/velocity_command
ros2 topic hz /mavros/setpoint_velocity/cmd_vel_unstamped

# Verify coordinator is working
ros2 topic echo /velocity_coordinator/active_source
```

## If LiDAR Still Not Working

1. Check AirSim is publishing sensor data:
```bash
ros2 node info /airsim_node
```

2. Try alternative topic names:
```bash
ros2 topic list | grep -i lidar
ros2 topic list | grep -i laser
ros2 topic list | grep -i point
```

3. Verify sensor is enabled in UE5:
- Check if LiDAR sensor component is attached to drone in Unreal
- Verify sensor is not disabled in blueprint

## Summary

The main issue was the keep-alive node hijacking velocity commands. The new system uses a priority-based velocity coordinator that allows search patterns to take control when active, while still maintaining OFFBOARD mode through lower-priority keep-alive commands.