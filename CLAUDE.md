# SAR Drone Project - Claude Memory
*Last Updated: August 8, 2025 (Afternoon)*

## Project Overview
MSc Thesis project: Autonomous Search and Rescue Drone System using ROS2, PX4, AirSim, and UE5.
Deadline: August 15, 2025 (7 days remaining)

## 🚨 IMPORTANT: Two Flight Control Modes Available

### Mode Selection (Ask at start of session):
1. **SimpleFlight Mode** - Pure AirSim, no PX4 needed, smooth demos
2. **PX4 Mode** - Realistic autopilot, deployable to real drone

Use `./airsim_native_workspace/switch_mode.sh` to toggle between modes.

## Current System State

### Working Components ✅
- Grid search pattern execution
- Web interface at http://localhost:5000 with HD video (1280x720)
- Vision system (YOLO detection with bounding boxes)
- Home position auto-saving (at >2m altitude with 2s delay)
- Velocity coordinator priority system
- Manual control via smart_keep_alive_node
- **RTH (Return to Home) - FIXED!** Drone correctly flies toward home
- Web control panel with separate altitude/rotation controls
- Yaw control for camera rotation
- **Hybrid position control** - Smooth, fast movements without oscillation
- **Practical LLM** - Natural language commands tested and working
- **Speed optimized** - 4.0 m/s movement for responsive control
- **Pattern System** - Expanding square, spiral, and zigzag patterns implemented
- **Pattern Visualizer** - Real-time matplotlib map showing drone movement
- **Pattern Monitor** - Terminal-based pattern execution monitoring

### Fixed Issues ✅

#### August 4
1. **RTH Navigation** - Used simple commands as reference, removed incorrect transformation
2. **RTH Cancellation** - Any manual command now cancels RTH
3. **Search Pattern Conflict** - Search aborts when RTH activates
4. **Home Position Auto-Save** - Added 2s delay for stable position
5. **Web Interface** - Added ascend/descend and rotate controls
6. **Camera Quality** - Upgraded to 1920x1080 @ 30 FPS in AirSim

### Remaining Issues ⚠️
1. **Pattern Execution**
   - Initial oscillation observed during pattern testing
   - Fixed by reducing waypoints and adding delays
   - May need further tuning for different pattern sizes

3. **PX4 Integration**
   - PX4 connects before UE5 Play button (should wait for simulator)
   - Preflight check failures after running launch scripts
   - Emergency stop causes velocity coordinator issues

4. **Minor Bugs**
   - Position estimator confusion when using backspace in AirSim
   - Matplotlib threading warnings (fixed with TkAgg backend)

### Progress Updates

#### August 8 (Afternoon) - SimpleFlight Tracker & Safety Improvements
1. **Created Real-time Position Tracker**
   - Built drone_tracker.py for live visualization
   - Shows XY position map with path history
   - Altitude graph with time series
   - Total distance traveled calculator
   - Fixed matplotlib deque slicing issues
   
2. **Integrated Tracker into START_HERE.sh**
   - Option 4: Position tracker only (monitor manual flight)
   - Option 5: Square pattern with live tracking
   - Option 6: Any pattern with background tracker
   
3. **Enhanced Landing Safety**
   - Created safe_pattern_runner.py with guaranteed landing
   - Added emergency stop handlers (Ctrl+C protection)
   - Gradual landing sequence: hover → 5m → ground
   - Updated all patterns to include enableApiControl(False)
   - Added try/finally blocks to ensure landing on crashes
   
4. **SimpleFlight System Updates**
   - Fixed tracker visualization errors
   - Added circle pattern to safe runner
   - Improved error handling across all scripts
   - Made all scripts executable

#### August 7 (Evening) - SimpleFlight Discovery & AirSim Native
1. **Discovered AirSim Settings Issue**
   - System was using PX4Multirotor mode (requires PX4 SITL)
   - Camera named "front_center" not "0", vehicle named "PX4" not "Drone1"
   - This explained why moveOnPathAsync wasn't working smoothly
   
2. **Created AirSim Native Workspace**
   - New workspace at `/airsim_native_workspace/`
   - Implemented SimpleFlight mode (no PX4 needed!)
   - Created mode switcher script to toggle between PX4 and SimpleFlight
   - Built complete demo suite with smooth patterns
   
3. **Analyzed Cosys AirSim path.py**
   - Found they use moveOnPathAsync with dynamic lookahead
   - Lookahead = velocity + velocity/2 for smooth turns
   - Implemented same approach in our native executors
   
4. **Fixed Pattern Execution Issues**
   - Created precision_pattern_executor with sharp turn detection
   - Increased speeds: 5m/s cruise, 2m/s turns
   - Added approach/departure waypoints for smooth turns
   - Fixed velocity coordinator to accept pattern commands

#### August 7 (Afternoon) - MAVROS Fix & Pattern Improvements
1. **Fixed MAVROS Launch Issue**
   - Updated launch_complete_pattern_system.sh to use same MAVROS command as working scripts
   - Changed from `ros2 launch mavros px4.launch` with wrong ports
   - To: `ros2 run mavros mavros_node --ros-args --param fcu_url:='udp://:14550@127.0.0.1:14540'`
   - All launch scripts now use consistent MAVROS configuration

2. **Improved Pattern Generation**
   - Fixed overlapping square pattern - removed redundant return to start
   - Added sharp 90° turns to zigzag with intermediate waypoints
   - Implemented yaw control - drone faces direction of travel
   - Added new lawnmower pattern for systematic coverage
   - Created improved_pattern_generator.py and improved_pattern_executor.py

3. **Fixed Visualization Issues**
   - Created terminal_map.py - ASCII visualization without matplotlib
   - No more focus stealing or matplotlib version conflicts
   - Real-time drone tracking in terminal with grid display

4. **Attitude-Based Flight Control**
   - Created attitude_pattern_executor.py using MAVROS setpoint_raw
   - Drone rotates to face direction BEFORE moving (like real flying)
   - No more sideways sliding - uses forward flight with yaw control
   - Different speeds: cruise (3m/s) and approach (1m/s)
   - Proper control modes: rotating, moving, approaching, idle

#### August 6 (Evening) - Pattern System Implementation
1. **Implemented Complete Pattern System**
   - Created pattern_generator.py - Generates waypoints for various patterns
   - Created pattern_executor.py - Executes waypoint sequences
   - Created pattern_visualizer.py - Real-time matplotlib visualization
   - Created pattern_monitor.py - Terminal-based monitoring
   - Patterns: Expanding square, spiral, zigzag

2. **Fixed Pattern Oscillation Issues**
   - Reduced waypoints (removed intermediate points)
   - Added delays between commands (min 1s)
   - Increased position threshold to 2.0m
   - Added stabilization time at each waypoint

3. **Created Pattern Test Scripts**
   - launch_complete_pattern_system.sh - Launches all nodes + patterns
   - test_pattern_with_map.sh - Pattern testing with visualization
   - test_pattern_safe.sh - Smaller patterns for testing
   - pattern_test_sequence.sh - Automated test of all patterns

4. **Pattern Visualization Tools**
   - Real-time map showing drone path (blue line)
   - Planned waypoints (green dashed line)
   - Current position (red dot) and target (yellow dot)
   - Alternative terminal monitor for systems without GUI

5. **Fixed Vision System Launch**
   - Corrected to use: `ros2 launch drone_vision_interpreter vision_pipeline.launch.py`
   - Added camera topic parameter for AirSim

6. **Discovered MAVROS Launch Issue**
   - ros2 run mavros not working properly
   - Need to use ros2 launch instead

#### August 6 (Afternoon) - Position Control & Practical LLM
1. **Created Position-Based Movement Controller**
   - Precise movements: "go forward 5 meters" now possible
   - Supports relative movements (forward/back/left/right/up/down X meters)
   - Supports absolute positioning (goto X,Y,Z)
   - Uses `/mavros/setpoint_position/local` for accurate control

2. **Implemented Practical LLM Controller**
   - Works with actual system capabilities (no false promises)
   - Parses natural language into position commands
   - Provides helpful feedback for unsupported requests
   - Integrated with position controller for precise movements

3. **Launch Scripts Created**
   - `launch_complete_system.sh` - All nodes with proper delays
   - `launch_practical_system.sh` - Position-based control system
   - `test_position_control.sh` - Demo sequence for testing

4. **System Architecture Improvements**
   - Moved from velocity-based to position-based control
   - Better separation of concerns (position controller handles movement)
   - LLM focuses on understanding intent, not low-level control

5. **Fixed QoS Compatibility Issues**
   - Added proper QoS profiles for MAVROS topics (BEST_EFFORT reliability)
   - Position controller and LLM nodes now receive pose data correctly
   - Created dedicated build script (`build_system.sh`)

6. **Hybrid Velocity-Position Controller**
   - Created hybrid controller to fix oscillation issues
   - Converts position targets to velocity commands
   - Works with existing velocity coordinator (priority 2)
   - Smooth approach with no overshoot
   - Speed optimized: 4.0 m/s max velocity, 3m slow-down zone
   - Tested and working with all movement commands including small distances

#### August 5 - LLM Integration & Bug Fixes
1. **Fixed Grid Search Transformation**
   - Applied same coordinate fix as RTH (direct X→X, Y→Y mapping)
   - Grid search now flies proper lawn mower pattern instead of diagonal

2. **Web Interface RTH Fix**
   - Fixed RTH button - now publishes to `/rth_command` with `'rth'`
   - Previously sent wrong command to wrong topic

3. **LLM Integration Setup**
   - OpenAI API key configured
   - Updated enhanced_llm_controller.py to use new OpenAI client syntax
   - Created test_llm_integration.sh script
   - LLM successfully parses natural language to drone commands

4. **System Testing**
   - Confirmed RTH works correctly via manual commands
   - Vision system (YOLO) working with bounding boxes
   - Web interface fully functional with all controls
   - LLM controller ready but needs full integration testing

#### August 4
1. **Fixed RTH Coordinate System**
   - Analyzed working simple commands as reference
   - Removed incorrect transformation (was negating and swapping)
   - Now uses direct mapping: X→X, Y→Y (no transformation needed)
   
2. **Enhanced Web Interface**
   - Added separate Altitude controls (Ascend/Descend)
   - Added Rotation controls (Yaw Left/Right)
   - Increased video resolution to HD (1280x720)
   - Improved control layout with grouped sections

3. **Fixed System Conflicts**
   - RTH can be cancelled by any manual command
   - Search pattern aborts when RTH is activated
   - Added RTH command listener to grid search node
   - Home position auto-saves after stable hover

4. **Prepared LLM Integration**
   - Created enhanced_llm_controller.py
   - Can parse natural language: "Find 3 people near position (10,20,5)"
   - Integrates with existing search and vision systems
   - Ready for testing with OpenAI API

### Key Commands
```bash
# Build system (run after any code changes)
./build_system.sh

# Launch system
cd /home/mbs/SAR-Drones-MSc-Thesis
./launch_hybrid_system.sh  # Recommended - hybrid control (no oscillation)

# Arm & Takeoff
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once

# Movement
ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward/backward/left/right/up/down/stop'" --once

# RTH (WORKING!)
ros2 topic pub /rth_command std_msgs/msg/String "data: 'rth'" --once

# Search Pattern
ros2 topic pub /search_command std_msgs/msg/String "data: 'search'" --once

# Yaw Control
ros2 topic pub /simple_command std_msgs/msg/String "data: 'yaw_left'" --once
ros2 topic pub /simple_command std_msgs/msg/String "data: 'yaw_right'" --once
```

### Important Files
- Progress reports: /home/mbs/SAR-Drones-MSc-Thesis/Updates/
- Main workspace: /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/
- AirSim settings: ~/Documents/AirSim/settings.json

### Test Scripts Available
```bash
# Launch Scripts
./launch_hybrid_system.sh           # RECOMMENDED - Hybrid control (no oscillation)
./launch_complete_pattern_system.sh # Complete system with patterns + visualization
./launch_complete_system.sh         # All nodes with proper delays
./launch_practical_system.sh        # Position-based control & practical LLM
./launch_position_only.sh           # Position control only (may oscillate)
./launch_with_vision.sh             # Basic system with YOLO detection

# Pattern Scripts
./test_pattern_with_map.sh          # Pattern testing with real-time map
./test_pattern_safe.sh              # Small patterns for safe testing
./pattern_test_sequence.sh          # Automated test of all patterns
./launch_drone_map.sh               # Just the map visualizer
./launch_pattern_monitor.sh         # Terminal-based pattern monitor

# Test Scripts
./test_rth_fixed.sh                 # Automated RTH test
./monitor_rth.sh                    # Visual monitoring dashboard
./test_auto_home.sh                 # Test auto home position saving
./test_position_control.sh          # Position-based movement demo
./test_llm_integration.sh           # Tests natural language commands
./test_llm_movement.sh              # Test LLM movement phrases
./monitor_velocity.sh               # Monitor velocity coordinator status
```

### Pattern Commands
```bash
# Generate patterns
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:10,3'" --once
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:20,5'" --once
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:30,30,5'" --once

# Control execution
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'pause'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'resume'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once

# Monitor
ros2 topic echo /pattern_status
ros2 topic echo /pattern_waypoints
```

### SimpleFlight Mode (NEW - August 7) 🚀

#### What is SimpleFlight?
Native AirSim control without PX4/MAVROS - direct, simple, smooth flight control.

#### Location
All SimpleFlight scripts in: `/home/mbs/SAR-Drones-MSc-Thesis/airsim_native_workspace/`

#### Key Advantages
- **No PX4 needed** - Just run UE5 and Python scripts
- **Smooth flight** - Uses AirSim's moveOnPathAsync with lookahead
- **Simple setup** - No MAVROS, no complex launchers
- **Fast testing** - One command to fly patterns
- **Better for demos** - Professional-looking smooth curves

#### Quick Start (SimpleFlight)
```bash
# 1. Switch to SimpleFlight mode
cd /home/mbs/SAR-Drones-MSc-Thesis/airsim_native_workspace
./switch_mode.sh  # Select option 2

# 2. Restart UE5 and press PLAY

# 3. Test connection
python3 scripts/test_connection.py

# 4. Run demos
python3 scripts/hover_test.py              # Basic hover
python3 patterns/smooth_square.py          # Smooth square pattern
python3 patterns/search_rescue_pattern.py  # Full SAR mission
```

#### SimpleFlight Scripts
```bash
# Quick start menu (UPDATED with web interface!)
./START_HERE.sh                 # Now includes option 10 for web interface

# Web Interface (NEW!)
./launch_web_interface.sh       # Launch web control at http://localhost:5001

# Individual tests
scripts/test_connection.py      # Test AirSim connection
scripts/hover_test.py           # Basic flight test

# Pattern scripts
patterns/smooth_square.py       # Smooth square with lookahead
patterns/search_rescue_pattern.py # Professional SAR pattern
patterns/square_with_tracker.py # Square with live position tracking
patterns/safe_pattern_runner.py # Failsafe patterns with guaranteed landing

# Visualization
visualizer/drone_tracker.py     # Real-time position tracker

# Vision integration
vision/camera_test.py           # Test camera feed
vision/camera_feed_yolo.py      # YOLO detection on camera feed

# Web Interface
web_interface/simpleflight_web_server.py  # Flask web server for SimpleFlight control
```

#### Key Differences (SimpleFlight vs PX4)
| Feature | SimpleFlight | PX4 Mode |
|---------|-------------|----------|
| **Setup** | Just UE5 | PX4 + MAVROS + ROS2 |
| **Camera** | "0" | "front_center" |
| **Vehicle** | "Drone1" | "PX4" |
| **Control** | Direct API | Through MAVROS |
| **Smoothness** | Very smooth | Can be jerky |
| **Deployment** | Simulation only | Can deploy to real drone |

### Next Session Plan (August 9) 🎯

#### Priority 1: Choose Flight Mode
**ASK USER AT START**: "Which mode today - SimpleFlight (demos) or PX4 (realistic)?"

#### Priority 2: Complete LLM Integration
1. **Pattern LLM Bridge Created**
   - Already have pattern_llm_bridge.py
   - Need to test integration
   - Update practical_llm to recognize pattern commands

2. **Test Complete System**
   - Full integration test with patterns
   - Natural language pattern commands
   - Ensure smooth execution

#### Priority 3: Demo Preparation
1. **Create Compelling Demo Scenarios**
   - "Missing hiker" scenario with grid search
   - Precision navigation demo (go to specific coordinates)
   - Natural language control showcase
   
2. **Fix Remaining Issues**
   - PX4 startup timing (add UE5 wait)
   - Test emergency stop integration
   - Ensure stable performance for demo

#### Priority 4: Documentation & Video
1. **Record Demo Videos**
   - System startup procedure
   - Natural language control examples
   - Search and rescue scenario
   
2. **Prepare Thesis Materials**
   - System architecture diagrams
   - Performance metrics
   - Future work recommendations

### Time Remaining: 7 days
Focus areas:
- Days 6-7: Complete LLM integration and advanced features
- Days 8-9: System robustness and edge case handling
- Days 10-12: Documentation and demo recording
- Days 13-14: Final testing and presentation prep
- Day 15: Thesis submission

### Multi-Drone Implementation Strategy (If Time Allows)
**IMPORTANT**: If user says "okay let's add more drones":
1. **Create separate repository** - DO NOT modify current working system
2. **Copy entire SAR-Drones-MSc-Thesis** to new repo (e.g., SAR-Drones-Multi-Agent)
3. **Implement multi-drone features** in the copy only
4. **Original repo remains untouched** as safety net for demo

This ensures we always have a working single-drone system for the thesis demo.

### Key Decisions Made
#### August 6
1. **Hybrid Control Architecture** - Velocity-based position control prevents oscillation
2. **Speed Optimization** - 4.0 m/s max velocity for responsive movement
3. **Build System** - Separate build script from launch scripts

#### August 5
1. **Single Drone Focus** - Prioritizing perfection over multiple drones
2. **GPT-3.5-turbo** - Chosen over GPT-4 for cost/speed (30x cheaper, 3x faster)
3. **Direct Coordinate Mapping** - Fixed grid search to use X→X, Y→Y like RTH

### Status Summary (August 8 Afternoon/Evening)
✅ **COMPLETED TODAY**:
- Created real-time drone position tracker with matplotlib visualization
- Fixed tracker deque slicing errors for proper animation
- Integrated tracker into START_HERE.sh menu (options 4-6)
- Created safe_pattern_runner.py with guaranteed landing
- Added emergency stop handlers and gradual landing sequence
- Updated all patterns to release API control after landing
- Added circle pattern option to safe runner
- Improved error handling with try/finally blocks
- **NEW: Created SimpleFlight Web Interface!**
  - Full web control at http://localhost:5001
  - Live camera streaming with HUD overlay
  - Pattern execution via web (square, spiral, zigzag, search)
  - Real-time telemetry (altitude, speed, position, flight time)
  - Manual control with keyboard support (WASD + QE + ZC)
  - Emergency stop and RTH functionality
  - No ROS2/MAVROS needed - works directly with AirSim

✅ **COMPLETED YESTERDAY (August 7)**:
- Fixed MAVROS launch issue across all scripts
- Discovered and fixed AirSim settings (PX4 vs SimpleFlight)
- Created complete AirSim native workspace
- Implemented smooth pattern execution with moveOnPathAsync
- Fixed pattern overshooting with precision executor
- Added SimpleFlight mode for easy demos
- Built mode switcher for PX4/SimpleFlight toggle
- Camera system properly configured

✅ **TWO COMPLETE SYSTEMS READY**:
1. **SimpleFlight System** (For demos)
   - No PX4/MAVROS needed
   - Smooth flight with lookahead
   - One-command patterns
   - Perfect for videos

2. **PX4 System** (For realism)
   - Full autopilot simulation
   - Deployable to real drone
   - Industry standard
   - Complete feature set

🚧 **TODO Before Demo** (7 days remaining):
1. Test and complete LLM integration
2. Create demo scenarios
3. Record demonstration videos
4. Prepare thesis documentation

⚠️ **Known Issues**:
1. Must choose correct mode (SimpleFlight vs PX4) in settings.json
2. Camera name differs between modes ("0" vs "front_center")
3. LLM integration needs testing with both modes

### PX4 Restart Procedure
```bash
# Kill PX4
pkill -9 px4

# Clean state
rm -rf /tmp/px4*
rm -rf ~/.ros/log/*

# Restart
cd ~/PX4-Autopilot
make px4_sitl_default none_iris

# Wait for "Waiting for simulator..." then press Play in UE5
```