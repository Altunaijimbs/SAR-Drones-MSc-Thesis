# Complete Testing Workflow for SAR Drone System

## System Overview
The system uses both camera-based (YOLO) and LiDAR-based obstacle detection with sensor fusion for robust obstacle avoidance during search patterns.

## Step-by-Step Launch Procedure (MUST FOLLOW THIS ORDER!)

### Terminal 1: Unreal Engine 5.5
```bash
# Start Unreal Editor with the Cosys-AirSim project
# Press 'Play' button to start simulation
# Keep this running throughout
```

### Terminal 2: PX4 SITL
```bash
cd ~/PX4-Autopilot
make px4_sitl none_iris
```
Wait for: `pxh>` prompt before proceeding

### Terminal 3: AirSim ROS2 Wrapper
```bash
cd /home/mbs/Desktop/airsim/Cosys-AirSim/ros2
source install/setup.bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```
Wait for: `AirsimROSWrapper Initialized!` message

### Terminal 4: MAVROS (UDP Connection - CRITICAL!)
```bash
ros2 run mavros mavros_node --ros-args \
  --param fcu_url:="udp://:14550@127.0.0.1:14540" \
  --param target_system_id:=1 \
  --param target_component_id:=1
```
Wait for: `CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot`

### Terminal 5: Keep-Alive Node (CRITICAL - Maintains OFFBOARD mode)
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
ros2 run llm_controller keep_alive_node
```
This publishes continuous velocity commands at 20Hz to maintain OFFBOARD mode

### Terminal 6: Arm Drone and Set Mode
```bash
# Wait for all nodes to be running, then:
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Make drone takeoff to safe altitude:
ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once
# Wait 3-5 seconds, then:
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once
```

### Terminal 7: Vision System
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
ros2 launch drone_vision_interpreter vision_pipeline.launch.py \
  camera_topic:=/airsim_node/PX4/front_center_Scene/image
```

### Terminal 8: LLM Controller
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
ros2 launch llm_controller llm_controller.launch.py \
  velocity_topic:=/llm/velocity_command
```

### Terminal 9: Search Patterns with Obstacle Avoidance
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
# First, let's check if launch file exists and copy it to correct location
cp src/search_patterns/launch/*.launch.py install/search_patterns/share/search_patterns/launch/
# Then launch
ros2 launch search_patterns search_with_avoidance.launch.py
```

## Verification Steps

### 1. Check All Topics Are Active
```bash
# Terminal 9
ros2 topic list | grep -E "(airsim|drone|mavros)"
```

Expected topics:
- `/airsim_node/PX4/front_center_Scene/image` - Camera feed
- `/airsim_node/PX4/LidarSensor1/point_cloud` - LiDAR data
- `/drone/obstacles` - Vision-based obstacles
- `/drone/lidar_obstacles` - LiDAR-based obstacles
- `/drone/scene_description` - Scene understanding
- `/mavros/setpoint_velocity/cmd_vel` - Final velocity commands

### 2. Verify Sensor Data
```bash
# Check camera
ros2 topic hz /airsim_node/PX4/front_center_Scene/image

# Check LiDAR
ros2 topic hz /airsim_node/PX4/LidarSensor1/point_cloud

# Check obstacle detection
ros2 topic echo /drone/obstacles --once
ros2 topic echo /drone/lidar_obstacles --once
```

## Testing Commands

### Send Natural Language Commands (Terminal 10)
```bash
# Basic movements
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'fly forward'" --once
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'fly to the right'" --once
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'search for people'" --once

# Grid search
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'search the area with grid pattern'" --once
```

### Monitor System
```bash
# View camera with bounding boxes:
ros2 run rqt_image_view rqt_image_view
# Select: /drone/vision/debug_image

# Check scene descriptions:
ros2 topic echo /drone/scene_description

# Monitor drone state:
ros2 topic echo /mavros/state

# Monitor obstacle avoidance:
ros2 topic echo /drone/obstacle_avoidance_active
```

## Important Coordinate Transformation

### CRITICAL - Coordinate Systems:
- **ROS/MAVROS**: X-forward, Y-left, Z-up
- **AirSim/UE4**: X-right, Y-forward, Z-up

### Transformation (handled in llm_controller_node.py):
```
ROS forward (X) → UE forward (-Y)
ROS right (Y) → UE right (X)
ROS up (Z) → UE up (Z)
```

### Key Topics:
- Camera: `/airsim_node/PX4/front_center_Scene/image`
- LiDAR: `/airsim_node/PX4/LidarSensor1/point_cloud`
- Velocity: `/mavros/setpoint_velocity/cmd_vel_unstamped`
- LLM input: `/llm/command_input`
- Scene info: `/drone/scene_description`

## Troubleshooting

### Common Issues and Solutions:

1. **PX4 won't arm**: 
   ```bash
   # In PX4 console (Terminal 2):
   commander arm -f
   ```

2. **Drone auto-disarms**: 
   - Ensure keep_alive_node is running (Terminal 5)
   - Check: `ros2 topic hz /mavros/setpoint_velocity/cmd_vel_unstamped`

3. **MAVROS won't connect**: 
   - Kill all mavros processes: `killall -9 mavros_node`
   - Retry with UDP connection (Terminal 4)

4. **Launch file not found error**:
   ```bash
   # Copy launch files to install directory:
   cd ~/SAR-Drones-MSc-Thesis/ros2_ws
   mkdir -p install/search_patterns/share/search_patterns/launch
   cp src/search_patterns/launch/*.py install/search_patterns/share/search_patterns/launch/
   ```

5. **No LiDAR data**:
   - Check settings.json has LidarSensor1 enabled
   - Verify topic: `ros2 topic echo /airsim_node/PX4/LidarSensor1/point_cloud`

## Build Commands

After making changes:
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
colcon build --packages-select <package_name>
source install/setup.bash
```

## Shutdown Sequence
1. Ctrl+C all ROS2 nodes
2. Stop PX4 SITL (Terminal 2)
3. Stop Unreal Engine Play mode (Terminal 1)

## Package Dependencies
- drone_interfaces: Custom messages
- drone_vision_interpreter: YOLO vision
- llm_controller: Natural language processing
- search_patterns: Grid search and obstacle avoidance
- Python: numpy==1.26.4, opencv-python==4.8.1.78, ultralytics (YOLO)