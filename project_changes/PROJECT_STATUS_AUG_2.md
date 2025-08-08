# SAR Drone Project Status - August 2, 2025

## Today's Major Achievements

### 1. Fixed Critical Velocity Command Conflict ✅
**Problem**: Original keep_alive_node was hijacking all velocity commands, stopping search patterns after 3 seconds.

**Solution**: 
- Created `smart_keep_alive_node.py` that publishes to separate topic
- Implemented `velocity_coordinator.py` with priority system:
  ```
  Emergency (99) > RTH (4) > Search (3) > Avoidance (2) > LLM (1) > Keep-alive (0)
  ```

### 2. Resolved QoS Compatibility Issues ✅
**Problem**: Grid search couldn't receive MAVROS position data due to QoS mismatch.

**Solution**: Created `fixed_grid_search.py` with proper QoS settings matching MAVROS.

### 3. Implemented Return to Home (RTH) ✅
**Feature**: Stop command now returns drone to starting position instead of just hovering.
- `return_to_home_node.py` saves initial position and navigates back
- Integrated with velocity coordinator at priority 4

### 4. Enhanced Obstacle Avoidance ✅
- Created vision-based obstacle avoidance
- Implemented LiDAR obstacle detection (pending sensor activation)
- Created sensor fusion module for robust avoidance

## Current System Architecture

### Core Nodes
1. **PX4 SITL** - Flight controller simulation
2. **AirSim on UE5.5** - Physics and sensor simulation
3. **MAVROS** - ROS2-PX4 bridge (UDP connection)
4. **Smart Keep-Alive** - Maintains OFFBOARD mode without interference
5. **Velocity Coordinator** - Priority-based command arbitration
6. **Vision System** - YOLO-based object detection
7. **LLM Controller** - Natural language processing
8. **Fixed Grid Search** - Search pattern execution with proper QoS
9. **Return to Home** - Emergency return functionality

### Key Topics
- Camera: `/airsim_node/PX4/front_center_Scene/image`
- LiDAR: `/airsim_node/PX4/LidarSensor1/point_cloud` (not publishing - needs investigation)
- Final velocity: `/mavros/setpoint_velocity/cmd_vel_unstamped`
- Coordinator status: `/velocity_coordinator/active_source`
- Search commands: `/search_command`
- Stop/RTH: `/stop_command`

## Correct Launch Sequence

1. **UE5.5 with AirSim** - Press Play
2. **PX4 SITL**: `make px4_sitl none_iris`
3. **AirSim Wrapper**: `ros2 launch airsim_ros_pkgs airsim_node.launch.py`
4. **MAVROS**: `ros2 run mavros mavros_node --ros-args --param fcu_url:="udp://:14550@127.0.0.1:14540"`
5. **Smart Keep-Alive**: `ros2 run llm_controller smart_keep_alive_node`
6. **Velocity Coordinator**: `ros2 run search_patterns velocity_coordinator`
7. **Arm & Takeoff**: Standard MAVROS services
8. **Vision System**: `ros2 launch drone_vision_interpreter vision_pipeline.launch.py`
9. **LLM Controller**: `ros2 launch llm_controller llm_controller.launch.py`
10. **Fixed Grid Search**: `ros2 run search_patterns fixed_grid_search`
11. **Return to Home**: `ros2 run search_patterns return_to_home`

## Commands That Work

### Search Commands
```bash
ros2 topic pub /search_command std_msgs/msg/String "data: 'search'" --once
ros2 topic pub /search_command std_msgs/msg/String "data: 'grid search'" --once
```

### Stop/Return Commands
```bash
ros2 topic pub /stop_command std_msgs/msg/String "data: 'stop'" --once
ros2 topic pub /stop_command std_msgs/msg/String "data: 'return home'" --once
```

### Natural Language
```bash
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'fly forward'" --once
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'search for people'" --once
```

## Known Issues

### 1. LiDAR Not Publishing
- Sensor configured in settings.json but topic not active
- Need to verify UE5.5 blueprint has LiDAR component
- Alternative: Depth camera can provide similar functionality

### 2. Coordinate System Notes
- Transform handled in all movement nodes:
  ```python
  # ROS to UE4
  ue_x = -ros_y  # ROS Y → UE -X
  ue_y = ros_x   # ROS X → UE Y
  ue_z = ros_z   # Same
  ```

## Next Steps (Priority Order)

1. **Test full SAR mission scenario** with grid search + RTH
2. **Implement spiral search pattern** (code structure ready)
3. **Create expanding square pattern**
4. **Develop vision-guided navigation** (fly to detected person)
5. **Fine-tune YOLO on SAR dataset** (Kaggle dataset available)
6. **Implement mission logging** for thesis data
7. **Create demo scenarios** for thesis presentation

## Files Modified Today

### New Files Created
- `/search_patterns/fixed_grid_search.py` - Grid search with QoS fix
- `/search_patterns/velocity_coordinator.py` - Priority-based velocity management
- `/search_patterns/return_to_home_node.py` - RTH functionality
- `/search_patterns/lidar_obstacle_avoidance.py` - LiDAR processing
- `/search_patterns/fusion_obstacle_avoidance.py` - Sensor fusion
- `/llm_controller/smart_keep_alive_node.py` - Non-interfering keep-alive

### Updated Files
- `/search_patterns/setup.py` - Added new nodes
- `/llm_controller/setup.py` - Added smart keep-alive
- Various launch files for proper integration

## Important Configuration

### Velocity Topics by Priority
1. `/emergency/velocity_override` - Emergency stop
2. `/rth/velocity_command` - Return to home
3. `/search_pattern/velocity_command` - Search patterns
4. `/mavros/setpoint_velocity/cmd_vel_unstamped_safe` - Obstacle avoidance
5. `/llm/velocity_command` - LLM/manual control
6. `/keepalive/velocity_command` - Keep-alive (lowest)

### Deadline: August 15, 2025
**Days Remaining**: 13

## Quick Test Tomorrow

```bash
# After launching all nodes, test with:
ros2 topic pub /search_command std_msgs/msg/String "data: 'search'" --once
# Wait for pattern to start, then:
ros2 topic pub /stop_command std_msgs/msg/String "data: 'stop'" --once
# Drone should return to start position
```

## Critical Insight
The main breakthrough was realizing the keep_alive_node was overriding all other commands. The velocity coordinator pattern with priority levels solved this elegantly while maintaining OFFBOARD mode safety.