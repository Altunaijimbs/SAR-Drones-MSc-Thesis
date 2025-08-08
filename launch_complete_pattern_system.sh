#!/bin/bash
# Comprehensive launch script for complete pattern system with visualization

echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║      COMPLETE PATTERN SYSTEM WITH VISUALIZATION LAUNCHER           ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""
echo "This script will launch:"
echo "  1. All core system nodes (AirSim, MAVROS, controllers)"
echo "  2. Pattern generation and execution nodes"
echo "  3. Real-time movement visualization"
echo "  4. Provide test commands for patterns"
echo ""
echo "Prerequisites:"
echo "  - PX4 running (make px4_sitl_default none_iris)"
echo "  - UE5 with AirSim environment loaded and Play button pressed"
echo ""
echo "Press Enter to start the complete system..."
read

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo ""
echo "═══ PHASE 1: LAUNCHING CORE SYSTEM ═══"
echo ""

# Terminal 1: AirSim ROS Wrapper
echo "[1/13] Starting AirSim ROS Wrapper..."
gnome-terminal --title="AirSim ROS" -- bash -c "source /home/mbs/Desktop/airsim/Cosys-AirSim/ros2/install/setup.bash && ros2 launch airsim_ros_pkgs airsim_node.launch.py; exec bash"
echo "      Waiting 7 seconds for AirSim initialization..."
sleep 7

# Terminal 2: MAVROS
echo "[2/13] Starting MAVROS..."
gnome-terminal --title="MAVROS" -- bash -c "ros2 run mavros mavros_node --ros-args --param fcu_url:='udp://:14550@127.0.0.1:14540'; exec bash"
echo "      Waiting 5 seconds for MAVROS connection..."
sleep 5

# Terminal 3: Velocity Coordinator
echo "[3/13] Starting Velocity Coordinator..."
gnome-terminal --title="Velocity Coordinator" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns velocity_coordinator; exec bash"
sleep 3

# Terminal 4: Hybrid Position Controller
echo "[4/13] Starting Hybrid Position Controller..."
gnome-terminal --title="Hybrid Position Controller" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns hybrid_position_controller; exec bash"
sleep 3

# Terminal 5: Smart Keep Alive
echo "[5/13] Starting Smart Keep Alive..."
gnome-terminal --title="Smart Keep Alive" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run llm_controller smart_keep_alive_node; exec bash"
sleep 3

# Terminal 6: Return to Home
echo "[6/13] Starting Return to Home..."
gnome-terminal --title="Return to Home" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns return_to_home; exec bash"
sleep 3

# Terminal 7: Grid Search
echo "[7/13] Starting Fixed Grid Search..."
gnome-terminal --title="Grid Search" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns fixed_grid_search; exec bash"
sleep 3

# Terminal 8: Practical LLM Controller
echo "[8/13] Starting Practical LLM Controller..."
gnome-terminal --title="LLM Controller" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run llm_controller practical_llm; exec bash"
sleep 3

# Terminal 9: Web Server
echo "[9/13] Starting Web Server..."
gnome-terminal --title="Web Server" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run sar_web_platform web_server; exec bash"
sleep 3

# Terminal 10: Vision System
echo "[10/13] Starting Vision System (YOLO Detection)..."
gnome-terminal --title="Vision System" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 launch drone_vision_interpreter vision_pipeline.launch.py camera_topic:=/airsim_node/PX4/front_center_Scene/image; exec bash"
sleep 3

echo ""
echo "═══ PHASE 2: LAUNCHING PATTERN SYSTEM ═══"
echo ""

# Terminal 11: Pattern Generator
echo "[11/13] Starting Pattern Generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns pattern_generator; exec bash"
sleep 3

# Terminal 12: Pattern Executor
echo "[12/13] Starting Pattern Executor..."
gnome-terminal --title="Pattern Executor" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns pattern_executor; exec bash"
sleep 3

# Terminal 13: Pattern Visualizer (Map)
echo "[13/13] Starting Pattern Visualizer (Map Window)..."
gnome-terminal --title="Pattern Map" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns pattern_visualizer; exec bash"
sleep 3

echo ""
echo "═══ PHASE 3: SYSTEM READY ═══"
echo ""
echo "All nodes launched! The system is ready for operation."
echo ""
echo "Web Interface available at: http://localhost:5000"
echo "Map visualization should be visible in a separate window"
echo ""
echo "═══ STEP 1: ARM AND TAKEOFF ═══"
echo ""
echo "Execute these commands in order:"
echo ""
echo "1. Set OFFBOARD mode:"
echo "   ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{custom_mode: 'OFFBOARD'}\""
echo ""
echo "2. Arm the drone:"
echo "   ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\""
echo ""
echo "3. Takeoff to 5 meters:"
echo "   ros2 topic pub /simple_command std_msgs/msg/String \"data: 'up'\" --once"
echo ""
echo "Wait for drone to reach stable hover (about 10 seconds)..."
echo ""
echo "═══ STEP 2: TEST PATTERNS ═══"
echo ""
echo "Pattern commands to try:"
echo ""
echo "▶ Small Expanding Square (8m sides, 2 loops):"
echo "  ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'expanding_square:8,2'\" --once"
echo "  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "▶ Small Spiral (12m radius, 4m spacing):"
echo "  ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'spiral:12,4'\" --once"
echo "  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "▶ Zigzag Pattern (15x15m, 5m spacing):"
echo "  ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'zigzag:15,15,5'\" --once"
echo "  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "═══ PATTERN CONTROL COMMANDS ═══"
echo ""
echo "▶ Pause current pattern:"
echo "  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'pause'\" --once"
echo ""
echo "▶ Resume pattern:"
echo "  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'resume'\" --once"
echo ""
echo "▶ Stop pattern:"
echo "  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'stop'\" --once"
echo ""
echo "═══ MONITORING ═══"
echo ""
echo "▶ Pattern status:"
echo "  ros2 topic echo /pattern_status"
echo ""
echo "▶ Current waypoints:"
echo "  ros2 topic echo /pattern_waypoints"
echo ""
echo "▶ Drone position:"
echo "  ros2 topic echo /mavros/local_position/pose"
echo ""
echo "▶ Vision detections (YOLO):"
echo "  ros2 topic echo /vision_detections"
echo "  ros2 topic echo /obstacle_detected"
echo ""
echo "The Pattern Map window shows:"
echo "  • Blue line: Actual drone path"
echo "  • Green dashed: Planned waypoints"
echo "  • Red dot: Current position"
echo "  • Yellow dot: Current target"
echo ""
echo "System is ready! Copy and paste the commands above to test."
echo ""