# Error Analysis and Fixes

## Root Cause Analysis

### 1. **Velocity Command Conflict** (PRIMARY ISSUE)
- **Keep-alive node** publishes to `/mavros/setpoint_velocity/cmd_vel_unstamped` at 20Hz
- **Search patterns** publish to same topic
- Keep-alive **overrides** search commands after 3 seconds
- Result: Drone stops mid-search pattern

### 2. **QoS Incompatibility** 
- MAVROS publishes position with `BEST_EFFORT` reliability
- Grid search expects `RELIABLE` 
- Result: No position data → No navigation

### 3. **Missing LiDAR Data**
- Topic `/airsim_node/PX4/LidarSensor1/point_cloud` not publishing
- Possible: Sensor disabled or different topic name

## Implemented Solutions

### 1. Priority-Based Velocity Coordination
Created a velocity coordinator that manages command priorities:

```
Emergency Stop (99) > Search Patterns (3) > Obstacle Avoidance (2) > LLM Commands (1) > Keep-Alive (0)
```

**New Architecture**:
```
Keep-Alive → /keepalive/velocity_command ↘
LLM → /llm/velocity_command              → Velocity Coordinator → /mavros/.../cmd_vel_unstamped
Search → /search_pattern/velocity_command ↗
```

### 2. Fixed QoS Settings
```python
# In fixed_grid_search.py
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Match MAVROS
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

### 3. Smart Keep-Alive Node
- Publishes to separate topic
- Won't interfere with active missions
- Maintains OFFBOARD mode safely

## Quick Fix Instructions

### Option 1: Use Fixed Components (Recommended)
```bash
# Terminal 5 - Use smart keep-alive
ros2 run llm_controller smart_keep_alive_node

# Terminal 9 - Launch fixed search system
ros2 launch search_patterns fixed_search_system.launch.py

# Terminal 10 - Send search command
ros2 topic pub /search_command std_msgs/msg/String "data: 'search'" --once
```

### Option 2: Manual Coordination
```bash
# Terminal 5 - Smart keep-alive
ros2 run llm_controller smart_keep_alive_node

# Terminal 6 - Velocity coordinator
ros2 run search_patterns velocity_coordinator

# Terminal 7 - Fixed grid search
ros2 run search_patterns fixed_grid_search

# Send command
ros2 topic pub /search_command std_msgs/msg/String "data: 'grid search'" --once
```

## Verification Steps

1. **Check Active Controller**:
```bash
ros2 topic echo /velocity_coordinator/active_source
```
Should show `Active: search` during search pattern

2. **Monitor Velocities**:
```bash
# Final output to MAVROS
ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped

# Search pattern commands
ros2 topic echo /search_pattern/velocity_command
```

3. **Verify Position Reception**:
```bash
# In another terminal while search is running
ros2 topic hz /mavros/local_position/pose
```

## Expected Behavior

1. Drone arms and takes off normally
2. Search command triggers grid pattern
3. Velocity coordinator shows "Active: search"
4. Drone completes full grid pattern
5. Returns to hover when complete

## If Issues Persist

1. **Kill old keep-alive**: `killall -9 keep_alive_node`
2. **Check topics**: `ros2 topic list | grep velocity`
3. **Force emergency stop**: `ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: true" --once`

## LiDAR Troubleshooting

```bash
# Find correct topic name
ros2 topic list | grep -E "(lidar|laser|point|cloud)"

# Check AirSim node info
ros2 node info /airsim_node | grep pub
```

The key insight: **Keep-alive was the villain!** The new system ensures search patterns have priority while maintaining safe OFFBOARD mode.