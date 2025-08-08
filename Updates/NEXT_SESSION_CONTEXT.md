# NEXT SESSION QUICK START CONTEXT
**Last Updated**: August 3, 2025
**Deadline**: August 15, 2025 (12 days left)

---

## 🚨 CRITICAL ISSUES TO SOLVE IMMEDIATELY

### 1. RTH FLIES WRONG DIRECTION
**Status**: NOT FIXED - Drone flies away from home position
**Where**: `/ros2_ws/src/search_patterns/search_patterns/return_to_home_node.py`

**Quick Test**:
```bash
# After drone at altitude, move it away then:
ros2 topic pub /stop_command std_msgs/msg/String "data: 'stop'" --once
# Watch Terminal 11 (RTH node) - it shows "Returning home... Distance: X.Xm"
# BUT drone flies wrong direction!
```

**Root Cause Hypothesis**:
- Coordinate transformation might be wrong
- Lines 210-217 apply same transform as grid search (which works)
- BUT: Maybe position calculation needs different handling?

**Debug Commands Ready**:
```bash
# Monitor what's happening:
ros2 topic echo /velocity_coordinator/active_source  # Should show "Active: rth"
ros2 topic echo /rth/velocity_command               # Check velocity values
ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped  # Final commands
```

### 2. PX4 CONNECTS BEFORE UE5 PLAY
**Status**: Connection timing issue
**Symptom**: PX4 shows "ready for takeoff" immediately instead of "waiting for simulator"

**Quick Check**:
```bash
cd ~/PX4-Autopilot
make px4_sitl none_iris
# Should see: "Waiting for simulator to accept connection on TCP port 4560"
# If not, connection is broken
```

**Already Tried**:
- Updated `~/Documents/AirSim/settings.json` with RC disable params
- Verified TCP port 4560 settings
- Launch order is correct in scripts

---

## ✅ WHAT'S WORKING

1. **Grid Search** - Pattern execution works perfectly
2. **Web Interface** - `ros2 run sar_web_platform web_server` → http://localhost:5000
3. **Vision System** - YOLO detection operational
4. **Home Position Saving** - Now saves at correct altitude (>2m)
5. **Velocity Coordinator** - Priority system working

---

## 🎯 QUICK LAUNCH SEQUENCE

### Option 1: With Web Interface
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis
./launch_with_web.sh
```

### Option 2: Manual Launch (for testing)
```bash
# Terminal 1: PX4 (FIRST!)
cd ~/PX4-Autopilot && make px4_sitl none_iris

# Terminal 2: UE5 - Press Play AFTER PX4 shows "waiting"

# Terminal 3: AirSim Wrapper
cd /home/mbs/Desktop/airsim/Cosys-AirSim/ros2 && source install/setup.bash
ros2 launch airsim_ros_pkgs airsim_node.launch.py

# Terminal 4: MAVROS
ros2 run mavros mavros_node --ros-args --param fcu_url:="udp://:14550@127.0.0.1:14540"

# Terminal 5: Core System
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws && source install/setup.bash
ros2 run llm_controller smart_keep_alive_node

# Terminal 6: Velocity Coordinator (BEFORE search patterns!)
ros2 run search_patterns velocity_coordinator

# Terminal 7: Arm & Takeoff
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once
sleep 5
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once

# Terminal 8-11: Other nodes as needed
```

---

## 🔧 COORDINATE SYSTEM REFERENCE

### CONFIRMED TRANSFORMATION (Used in working grid search):
```python
# ROS to UE4/AirSim (for velocity commands)
transformed_vel.linear.x = -vel_cmd.linear.y  # ROS Y → UE -X
transformed_vel.linear.y = vel_cmd.linear.x   # ROS X → UE Y
transformed_vel.linear.z = vel_cmd.linear.z   # ROS Z → UE Z
```

### WHERE IT'S USED:
- ✅ `fixed_grid_search.py` line 232 - WORKS
- ❌ `return_to_home_node.py` line 212 - DOESN'T WORK
- ✅ `smart_keep_alive_node.py` - Uses UE4 directly (Y=forward)
- ❓ `keep_alive_node.py` - Uses ROS directly (X=forward) to MAVROS

### KEY QUESTION: 
Does MAVROS do internal transformation or expect UE4 coordinates?

---

## 🧪 IMMEDIATE TEST PROCEDURE

### Test 1: Coordinate System Verification
```bash
# After basic launch (Terminals 1-6), test each direction:
ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once
# OBSERVE: Which direction in UE5? Forward or sideways?

ros2 topic pub /simple_command std_msgs/msg/String "data: 'right'" --once  
# OBSERVE: Which direction in UE5?

# Also monitor:
ros2 topic echo /keepalive/velocity_command
# See what values smart_keep_alive sends
```

### Test 2: RTH Direction Debug
```python
# Add this to return_to_home_node.py line 183 (before distance calc):
self.get_logger().info(f'Current: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}, {self.current_pose.position.z:.2f})')
self.get_logger().info(f'Home: ({self.home_position.x:.2f}, {self.home_position.y:.2f}, {self.home_position.z:.2f})')
self.get_logger().info(f'Delta: dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f}')

# After line 208 (before transform):
self.get_logger().info(f'Vel CMD: x={vel_cmd.linear.x:.2f}, y={vel_cmd.linear.y:.2f}')

# After line 214 (after transform):
self.get_logger().info(f'Transformed: x={transformed_vel.linear.x:.2f}, y={transformed_vel.linear.y:.2f}')
```

---

## 📁 KEY FILES TO CHECK

1. **RTH Logic**: `/home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/src/search_patterns/search_patterns/return_to_home_node.py`
2. **Working Reference**: `/home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/src/search_patterns/search_patterns/fixed_grid_search.py`
3. **Velocity Routing**: `/home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/src/search_patterns/search_patterns/velocity_coordinator.py`
4. **AirSim Config**: `~/Documents/AirSim/settings.json`

---

## 💡 HYPOTHESES TO TEST

### H1: Position vs Velocity Coordinates
- Maybe MAVROS position is in different frame than velocity?
- Test: Log raw position values and check if they match UE5 visualization

### H2: Velocity Coordinator Issue  
- Maybe RTH commands aren't reaching MAVROS?
- Test: `ros2 topic hz /mavros/setpoint_velocity/cmd_vel_unstamped` during RTH

### H3: Transform Should Be Different for RTH
- Maybe RTH needs inverse transform since it's position-based?
- Test: Try removing transformation in RTH (just use vel_cmd directly)

### H4: Home Position Wrong Frame
- Maybe saved position needs transformation too?
- Test: Log home position and verify it matches drone's actual location in UE5

---

## 🎯 DECISION POINTS

1. **Should we remove coordinate transform from RTH?**
   - Grid search needs it and works
   - But RTH might be different case

2. **Should we test with old keep_alive_node?**
   - It bypasses velocity coordinator
   - Might reveal if coordinator is the issue

3. **Should positions be transformed when saved?**
   - Currently saving raw MAVROS positions
   - Maybe need to transform before saving

---

## 🚀 IF EVERYTHING ELSE FAILS

### Nuclear Option 1: Copy Grid Search Logic
Since grid search WORKS, copy its exact movement logic:
```python
# In RTH, replace current movement with grid search style:
# Set home as a waypoint and use same navigation logic
```

### Nuclear Option 2: Use Global Position
```python
# Subscribe to /mavros/global_position/global
# Use GPS coordinates for RTH instead of local
```

### Nuclear Option 3: Simple Testing
```python
# Hardcode a simple velocity to test:
vel_cmd.linear.x = 1.0  # Just fly forward
# See which direction it goes in UE5
```

---

## 📝 REMEMBER
- **Deadline**: August 15 (12 days)
- **Priority**: Fix RTH, then document for thesis
- **Backup Plan**: If RTH unfixable, use "fly to waypoint" approach
- **Web Interface**: Already professional and working!

---

**CRITICAL**: Start next session by running Test 1 (coordinate verification) BEFORE trying to fix anything!