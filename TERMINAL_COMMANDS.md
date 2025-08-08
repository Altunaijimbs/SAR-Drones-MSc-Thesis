# Terminal Commands for Coordinate Testing
## Quick Reference Guide

---

## Terminal 1: PX4 SITL
```bash
cd ~/PX4-Autopilot
make px4_sitl none_iris
```
Wait for: `pxh> simulator connected`

---

## Terminal 2: Unreal Engine
1. Open UE5
2. Load SAR project  
3. Press PLAY
4. Minimize window

---

## Terminal 3: System Launch
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis
./test_fixed_system.sh
```
Wait for: `=== SYSTEM READY ===`

---

## Terminal 4: Coordinate Monitor
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash
python3 /home/mbs/SAR-Drones-MSc-Thesis/diagnose_coordinates.py
```

---

## Terminal 5: Test Commands (Main)
```bash
# Setup
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash

# Arm and takeoff
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once

# Wait 5 seconds for stable hover

# Test 1: Raw MAVROS X-axis (run for 2 seconds)
ros2 topic pub /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}}" -r 10 -t 2

# Stop
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once

# Test 2: Raw MAVROS Y-axis (run for 2 seconds)
ros2 topic pub /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 1.0, z: 0.0}}" -r 10 -t 2

# Stop
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once

# Test 3: Raw MAVROS Z-axis (run for 2 seconds)
ros2 topic pub /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 1.0}}" -r 10 -t 2

# Stop
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once

# Smart Keep-Alive Tests
ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once
# Wait, observe, then stop

ros2 topic pub /simple_command std_msgs/msg/String "data: 'right'" --once
# Wait, observe, then stop

ros2 topic pub /simple_command std_msgs/msg/String "data: 'backward'" --once
# Wait, observe, then stop

ros2 topic pub /simple_command std_msgs/msg/String "data: 'left'" --once
# Wait, observe, then stop
```

---

## Terminal 6: Transformation Tests (After creating coordinate_test.py)
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
source install/setup.bash

# Test no transformation
ros2 topic pub /test_command std_msgs/msg/String "data: 'no_transform'" --once

# Test current transformation
ros2 topic pub /test_command std_msgs/msg/String "data: 'transform_current'" --once

# Test negated transformation
ros2 topic pub /test_command std_msgs/msg/String "data: 'transform_negated'" --once
```

---

## Recording Template
Create `/home/mbs/SAR-Drones-MSc-Thesis/Updates/COORDINATE_TEST_RESULTS.md`:

```markdown
# Coordinate Test Results - August 3, 2025

## Raw MAVROS Tests
- MAVROS X=1.0 → Drone moved: _______
- MAVROS Y=1.0 → Drone moved: _______
- MAVROS Z=1.0 → Drone moved: _______

## Smart Keep-Alive Tests
- 'forward' → Drone moved: _______, MAVROS velocity: X=___ Y=___ Z=___
- 'right' → Drone moved: _______, MAVROS velocity: X=___ Y=___ Z=___
- 'backward' → Drone moved: _______, MAVROS velocity: X=___ Y=___ Z=___
- 'left' → Drone moved: _______, MAVROS velocity: X=___ Y=___ Z=___

## Transformation Tests
- No transform: ROS X=1.0 → Drone moved: _______
- Current transform: ROS X=1.0 → Drone moved: _______
- Negated transform: ROS X=1.0 → Drone moved: _______

## Conclusions
The correct transformation is: _______
```

---

## Quick Debug Commands

### Monitor what's being sent to MAVROS:
```bash
ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped
```

### Monitor drone position:
```bash
ros2 topic echo /mavros/local_position/pose --field pose.position
```

### Emergency stop:
```bash
ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: true" --once
```

### Land immediately:
```bash
ros2 topic pub /simple_command std_msgs/msg/String "data: 'down'" --once
```