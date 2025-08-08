# Context for Next Session - SAR Drone Project

## Session Date: August 2, 2025
**Deadline**: August 15, 2025 (13 days remaining)

## What We Accomplished Today

### 1. Diagnosed and Fixed Major Issues
- **Root Cause**: Original `keep_alive_node` was overriding all velocity commands after 3 seconds
- **Solution**: Created priority-based velocity coordination system
- **Result**: Search patterns now work without interference

### 2. Key Components Created
- `smart_keep_alive_node.py` - Non-interfering OFFBOARD maintenance
- `velocity_coordinator.py` - Priority-based command arbitration
- `fixed_grid_search.py` - Grid search with proper QoS settings
- `return_to_home_node.py` - Stop command returns to start position
- LiDAR and fusion obstacle avoidance modules (ready but LiDAR not publishing)

### 3. Current State
- ✅ Grid search pattern working
- ✅ Obstacle avoidance implemented (vision-based)
- ✅ Return to home on stop command
- ✅ Natural language control functional
- ❌ LiDAR not publishing (sensor issue)
- 🔄 Spiral and expanding square patterns pending

## Critical Information

### Velocity Command Priority (Highest to Lowest)
1. Emergency Stop (99)
2. Return to Home (4)
3. Search Patterns (3)
4. Obstacle Avoidance (2)
5. LLM/Manual Commands (1)
6. Keep-Alive (0)

### Coordinate Transformation (Used Everywhere)
```python
# ROS to UE4/AirSim
ue_x = -ros_y  # ROS right → UE left
ue_y = ros_x   # ROS forward → UE forward
ue_z = ros_z   # ROS up → UE up
```

### Key Topics
- Search commands: `/search_command`
- Stop/RTH: `/stop_command`
- Coordinator status: `/velocity_coordinator/active_source`
- Natural language: `/llm/command_input`

## To Start Tomorrow

### 1. Build Check
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
colcon build --packages-select llm_controller search_patterns
source install/setup.bash
```

### 2. Use QUICK_START_GUIDE.md
Follow the exact launch sequence - order matters!

### 3. Quick Test
```bash
# Start search
ros2 topic pub /search_command std_msgs/msg/String "data: 'search'" --once

# Stop and return
ros2 topic pub /stop_command std_msgs/msg/String "data: 'stop'" --once
```

## Next Priorities

### High Priority (For Demo)
1. Test complete SAR mission scenario
2. Implement spiral search pattern
3. Vision-guided navigation (fly to detected person)
4. Create demo video for thesis

### Medium Priority
1. Fix LiDAR sensor publishing
2. Fine-tune YOLO on SAR dataset
3. Mission logging system
4. Expanding square pattern

### Documentation Needed
1. Demo scenarios script
2. Performance metrics
3. Thesis integration points

## Known Issues
1. **LiDAR not publishing** - Check UE5.5 blueprint sensor attachment
2. **API key not set** - Using rule-based LLM fallback (works fine)

## Important Files
- Working launch sequence: `sar_drone_launch_sequence.txt`
- Error log: `errors` file in Debugging folder
- All new code in: `/home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/src/search_patterns/`

## Remember
- Always use `smart_keep_alive_node` NOT `keep_alive_node`
- Always launch `velocity_coordinator` before search patterns
- Use `fixed_grid_search` NOT regular `grid_search`
- MAVROS needs UDP connection parameters

The system is now functional for grid search demonstrations!