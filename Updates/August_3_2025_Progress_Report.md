# SAR Drone System Development Progress Report
## Date: August 3, 2025
## MSc Thesis - Search and Rescue Drone Platform

---

## Executive Summary

Today's session focused on addressing critical system issues identified from yesterday's work, implementing a web-based control interface, and troubleshooting various integration challenges. While significant progress was made in several areas, some core issues remain unresolved.

---

## Major Accomplishments

### 1. Fixed Return to Home (RTH) Node Home Position Storage

**Previous Issue**: The RTH node was saving the drone's position immediately upon initialization, often capturing ground-level coordinates (0 altitude) or intermediate positions during takeoff.

**Solution Implemented**:
- Modified `return_to_home_node.py` to implement intelligent home position saving
- Added altitude threshold check (minimum 2 meters)
- Added drone state verification (must be armed and in OFFBOARD mode)
- Implemented manual home position setting via `/set_home_position` topic
- Added visual confirmation with WARNING-level log when home is saved

**Code Changes**:
```python
# Added parameters
self.declare_parameter('min_altitude_for_home', 2.0)
self.declare_parameter('auto_set_home', True)

# Added state monitoring
self.state_sub = self.create_subscription(
    State,
    '/mavros/state',
    self.state_callback,
    10
)

# Intelligent home saving logic
if (self.auto_set_home and 
    not self.initial_position_saved and 
    self.current_pose is not None and
    self.is_armed and 
    self.mode == 'OFFBOARD' and
    self.current_pose.position.z >= self.min_altitude):
    self.save_home_position()
```

### 2. Developed Complete Web-Based Control Platform

**Features Implemented**:
- Modern dark-themed responsive web interface
- Real-time drone status monitoring (position, mode, battery, GPS)
- Live video streaming from drone camera (MJPEG)
- Mission control buttons (arm/disarm, takeoff/land, search patterns)
- Manual flight control with on-screen joystick
- Natural language command input
- System activity logging
- Keyboard shortcuts (arrow keys for movement, Ctrl+E for emergency stop)

**Technical Architecture**:
- Backend: Flask-based ROS2 node (`sar_web_platform`)
- Frontend: HTML5/CSS3/JavaScript with real-time updates
- Communication: RESTful API + MJPEG streaming
- Update rate: 2Hz for status polling

**File Structure Created**:
```
sar_web_platform/
├── sar_web_platform/
│   ├── __init__.py
│   └── web_server.py
├── templates/
│   └── index.html
├── static/
│   ├── css/
│   │   └── style.css
│   └── js/
│       └── app.js
├── scripts/
│   └── web_server (executable)
├── launch/
│   └── web_platform.launch.py
├── setup.py
├── package.xml
├── requirements.txt
└── README.md
```

### 3. Fixed Web Server Executable Issue

**Problem**: ROS2 couldn't find the web_server executable with "No executable found" error

**Root Cause**: ROS2 expects executables in `lib/<package_name>/` directory, not in `bin/`

**Solution**:
- Created executable script in `scripts/web_server`
- Modified setup.py to install to correct location
- Added proper data_files configuration for templates and static files

### 4. Updated Launch Scripts with Correct Order

**Issue Identified**: Launch order was incorrect - UE5 was started before PX4

**Correct Order**:
1. Start PX4 SITL (waits for simulator connection)
2. Press Play in Unreal Engine (connects to PX4)
3. Continue with other nodes

**Scripts Updated**:
- `launch_sar_system.sh` - Core system launch
- `launch_with_web.sh` - Includes web interface

---

## Ongoing Issues and Analysis

### 1. Return to Home Navigation Problem

**Symptoms**:
- Drone does not return to saved home position
- May fly in wrong direction or toward objects
- Appears to be coordinate system related

**Investigation Findings**:

**Coordinate System Analysis**:
- MAVROS provides positions in ROS coordinates (X=forward, Y=left, Z=up)
- AirSim/UE5 uses different coordinates (X=right, Y=forward, Z=up)
- Transformation is applied: `ue_x = -ros_y, ue_y = ros_x, ue_z = ros_z`

**Working Nodes Analysis**:
1. **Grid Search** - WORKS
   - Reads from `/mavros/local_position/pose`
   - Applies coordinate transformation before publishing
   - Publishes to `/search_pattern/velocity_command`

2. **Smart Keep-Alive** - WORKS
   - Uses UE4 coordinates directly (`linear.y` for forward)
   - Publishes to `/keepalive/velocity_command`

**RTH Implementation**:
- Uses same transformation as grid search
- Publishes to `/rth/velocity_command`
- Priority 4 in velocity coordinator (should override obstacle avoidance at priority 2)

**Potential Issues**:
1. Obstacle avoidance interference (though RTH has higher priority)
2. Incorrect velocity calculation or transformation
3. Emergency stop being sent simultaneously (FIXED - removed this)
4. Position coordinates may also need transformation when calculating direction

### 2. PX4 System Health Failures

**Symptoms**:
- PX4 reports "ready for takeoff" before UE5 Play is pressed
- System health check failures
- Possible connection issues between PX4 and AirSim

**Attempted Solutions**:
1. Updated AirSim settings.json with additional parameters:
   ```json
   "Parameters": {
       "NAV_RCL_ACT": 0,
       "NAV_DLL_ACT": 0,
       "COM_OBL_ACT": 1,
       "COM_RC_IN_MODE": 4,
       "COM_DISARM_LAND": -1,
       "COM_DISARM_PREFLIGHT": -1
   }
   ```

2. Verified connection settings:
   - TCP connection on port 4560 (correct)
   - Control ports configured
   - LockStep enabled

**Current Status**: Issue persists, may require cache clearing or fresh installation

### 3. Coordinate System Inconsistencies

**Core Challenge**:
Different nodes use different coordinate conventions:
- Old keep_alive_node: ROS coordinates to MAVROS directly
- Smart keep_alive_node: UE4 coordinates
- Grid search: ROS with transformation
- RTH: ROS with transformation
- LLM controller: Complex transformation logic

**Key Question**: Does MAVROS expect ROS or UE4 coordinates for velocity commands?

**Evidence suggests MAVROS expects UE4 coordinates** because:
- Grid search works with transformation
- Smart keep-alive works with UE4 coordinates directly

---

## System Architecture Summary

### Current Node Hierarchy:
1. **Simulation Layer**
   - Unreal Engine 5.5 with Cosys-AirSim
   - PX4 SITL (Software In The Loop)

2. **Bridge Layer**
   - AirSim ROS2 Wrapper
   - MAVROS (UDP connection)

3. **Control Layer**
   - Velocity Coordinator (priority arbitration)
   - Smart Keep-Alive (OFFBOARD maintenance)

4. **Application Layer**
   - Grid Search Pattern
   - Return to Home
   - Vision System (YOLO object detection)
   - LLM Controller (natural language)
   - Web Platform (user interface)

### Topic Flow:
```
User Commands → Application Nodes → Velocity Coordinator → MAVROS → PX4 → AirSim → UE5
```

### Velocity Priority System:
1. Emergency Stop (99)
2. Return to Home (4)
3. Search Patterns (3)
4. Obstacle Avoidance (2)
5. LLM/Manual Commands (1)
6. Keep-Alive (0)

---

## Next Steps and Recommendations

### Immediate Actions Needed:

1. **Coordinate System Testing**
   - Systematic test of each direction command
   - Monitor actual vs expected movement
   - Document transformation requirements

2. **RTH Debugging**
   - Add extensive logging to track:
     - Saved home position
     - Current position
     - Calculated direction vector
     - Transformed velocity command
   - Monitor velocity coordinator active source during RTH

3. **PX4 Connection Resolution**
   - Clear all caches (UE5 and AirSim)
   - Verify port configurations
   - Consider fresh PX4 installation

### Future Enhancements:

1. **Web Platform**
   - Add 3D visualization
   - Implement mission planning interface
   - Add multi-drone support
   - Create mobile-responsive version

2. **System Integration**
   - Standardize coordinate system across all nodes
   - Implement comprehensive error handling
   - Add telemetry recording
   - Create automated testing suite

---

## Technical Notes

### File Modifications Today:
1. `/search_patterns/return_to_home_node.py` - Home position logic
2. `/sar_web_platform/` - Complete package creation
3. `/launch_sar_system.sh` - Launch order fix
4. `/launch_with_web.sh` - Web integration
5. `~/Documents/AirSim/settings.json` - Parameter updates

### Dependencies Added:
- Flask >= 2.3.0
- flask-cors >= 4.0.0
- opencv-python >= 4.8.0
- numpy >= 1.24.0

### Known Working Commands:
```bash
# Grid search
ros2 topic pub /search_command std_msgs/msg/String "data: 'search'" --once

# Stop and RTH (currently buggy)
ros2 topic pub /stop_command std_msgs/msg/String "data: 'stop'" --once

# Manual control
ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once

# Natural language
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'fly forward and search'" --once
```

---

## Conclusion

Significant progress was made in creating a professional web-based control interface and fixing the home position saving logic. However, critical issues remain with the RTH navigation accuracy and PX4-AirSim connection stability. The coordinate system transformation between ROS and UE4/AirSim continues to be a source of complexity and potential bugs.

The project deadline is August 15, 2025 (12 days remaining), making it crucial to resolve these core issues quickly to allow time for testing and thesis documentation.

---

*Report compiled: August 3, 2025*
*Project: MSc Thesis - Autonomous SAR Drone System*
*Author: System Development Log*