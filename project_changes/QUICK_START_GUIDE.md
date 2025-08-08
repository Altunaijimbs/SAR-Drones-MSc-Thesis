# Quick Start Guide - SAR Drone System

## Prerequisites Check
- [ ] Unreal Engine 5.5 with Cosys-AirSim project loaded
- [ ] Terminal windows ready (need ~12 terminals)
- [ ] Working directory: `/home/mbs/SAR-Drones-MSc-Thesis/ros2_ws`

## Launch Sequence (EXACT ORDER - CRITICAL!)

### Terminal 1: Unreal Engine
- Open UE5.5 project
- Press **Play** button
- Keep running throughout session

### Terminal 2: PX4 SITL
```bash
cd ~/PX4-Autopilot
make px4_sitl none_iris
# Wait for: pxh>
```

### Terminal 3: AirSim ROS2 Wrapper
```bash
cd /home/mbs/Desktop/airsim/Cosys-AirSim/ros2
source install/setup.bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py
# Wait for: AirsimROSWrapper Initialized!
```

### Terminal 4: MAVROS (UDP!)
```bash
ros2 run mavros mavros_node --ros-args \
  --param fcu_url:="udp://:14550@127.0.0.1:14540" \
  --param target_system_id:=1 \
  --param target_component_id:=1
# Wait for: CON: Got HEARTBEAT, connected
```

### Terminal 5: Smart Keep-Alive ⚠️ NEW!
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
ros2 run llm_controller smart_keep_alive_node
# NOT the old keep_alive_node!
```

### Terminal 6: Velocity Coordinator ⚠️ NEW!
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
ros2 run search_patterns velocity_coordinator
```

### Terminal 7: Arm & Takeoff
```bash
# Set OFFBOARD mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"

# Arm drone
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Takeoff
ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once
# Wait 5 seconds
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once
```

### Terminal 8: Vision System
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
ros2 launch drone_vision_interpreter vision_pipeline.launch.py \
  camera_topic:=/airsim_node/PX4/front_center_Scene/image
```

### Terminal 9: LLM Controller
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
ros2 launch llm_controller llm_controller.launch.py \
  velocity_topic:=/llm/velocity_command
```

### Terminal 10: Fixed Grid Search ⚠️ NEW!
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
ros2 run search_patterns fixed_grid_search
# NOT the old grid_search!
```

### Terminal 11: Return to Home ⚠️ NEW!
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
ros2 run search_patterns return_to_home
```

## Test Commands (Terminal 12)

### Basic Movement Test
```bash
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'fly forward'" --once
```

### Grid Search Test
```bash
ros2 topic pub /search_command std_msgs/msg/String "data: 'search'" --once
```

### Stop and Return Home
```bash
ros2 topic pub /stop_command std_msgs/msg/String "data: 'stop'" --once
```

### Monitor Active Controller
```bash
ros2 topic echo /velocity_coordinator/active_source
```

## Troubleshooting

### If drone won't move during search:
```bash
# Check velocity coordinator
ros2 topic echo /velocity_coordinator/active_source
# Should show "Active: search" not "Active: keepalive"
```

### If old keep_alive is running:
```bash
killall -9 keep_alive_node
# Make sure using smart_keep_alive_node instead!
```

### Emergency stop:
```bash
ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: true" --once
```

## Key Changes from Original System
1. **smart_keep_alive_node** replaces keep_alive_node
2. **velocity_coordinator** manages command priorities
3. **fixed_grid_search** has proper QoS settings
4. **return_to_home** handles stop commands
5. All velocity commands go through coordinator

## Success Indicators
- Velocity coordinator shows correct active source
- Drone completes search patterns without stopping
- Stop command returns drone to start position
- No QoS warnings in terminals