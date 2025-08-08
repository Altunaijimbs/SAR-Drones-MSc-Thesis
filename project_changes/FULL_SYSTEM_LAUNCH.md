# Full System Launch Guide

## Required Components

### 1. PX4 SITL (Terminal 1)
```bash
cd ~/PX4-Autopilot
make px4_sitl none_iris
```

### 2. AirSim/Unreal Engine (Terminal 2)
```bash
cd ~/Desktop/airsim/Cosys-AirSim
./run.sh  # or launch your UE5.5 project
```

### 3. AirSim ROS2 Wrapper (Terminal 3)
```bash
cd ~/Desktop/airsim/Cosys-AirSim/ros2
source install/setup.bash
ros2 launch airsim_ros_wrapper airsim_node.launch.py
```

### 4. MAVROS (Terminal 4)
```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch.py fcu_url:="udp://:14540@127.0.0.1:14557"
```

### 5. Vision System (Terminal 5)
```bash
cd ~/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
ros2 launch drone_vision_interpreter vision_pipeline.launch.py
```

### 6. LLM Controller (Terminal 6)
```bash
cd ~/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
ros2 launch llm_controller llm_controller.launch.py
```

### 7. Search Patterns with Obstacle Avoidance (Terminal 7)
```bash
cd ~/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
ros2 launch search_patterns search_with_avoidance.launch.py
```

## System Architecture

```
Camera (AirSim) → Vision System → Object Detection → Obstacles
                                → Scene Description → LLM Controller
                                                   ↓
User Command → LLM Controller → Search Pattern Manager → Grid Search
                                                      ↓
                                              Velocity Commands
                                                      ↓
                                            Obstacle Avoidance
                                                      ↓
                                           Velocity Multiplexer
                                                      ↓
                                                   MAVROS
                                                      ↓
                                                  PX4 SITL
                                                      ↓
                                                   AirSim
```

## Testing Commands

### Basic Movement (Terminal 8)
```bash
# Test forward movement
ros2 topic pub /drone/command std_msgs/msg/String "data: 'fly forward'" --once

# Test obstacle avoidance
ros2 topic pub /drone/command std_msgs/msg/String "data: 'fly forward slowly'" --once
```

### Search Patterns
```bash
# Grid search
ros2 topic pub /drone/command std_msgs/msg/String "data: 'search the area with grid pattern'" --once

# Monitor obstacle avoidance
ros2 topic echo /drone/obstacle_avoidance_active
```

## Key Topics

- `/drone/command` - Send natural language commands
- `/drone/obstacles` - Detected obstacles from vision
- `/drone/scene_description` - Scene understanding
- `/mavros/setpoint_velocity/cmd_vel` - Final velocity to drone
- `/drone/obstacle_avoidance_active` - Avoidance status

## Safety Features

1. **Reactive Obstacle Avoidance**
   - 3m safety distance (starts avoidance)
   - 1.5m critical distance (emergency stop)
   - Lateral avoidance preferred over stopping

2. **Velocity Multiplexing**
   - Prioritizes safety commands
   - Timeout protection (0.5s)
   - Smooth transitions

3. **Coordinate Transform**
   - Handles ROS → UE4 conversion
   - Consistent across all modules