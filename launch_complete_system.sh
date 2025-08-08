#!/bin/bash
# Complete SAR Drone System Launch Script with Proper Delays
# This script launches all nodes with appropriate idle time between each

echo "╔══════════════════════════════════════════════════════╗"
echo "║     SAR DRONE SYSTEM - COMPLETE LAUNCH SCRIPT        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Prerequisites:"
echo "  ✓ PX4 is running and waiting for simulator"
echo "  ✓ UE5 is playing"
echo "  ✓ OPENAI_API_KEY is set (for LLM features)"
echo ""
echo "Press Enter when ready to start..."
read

# Check if OPENAI_API_KEY is set
if [ -z "$OPENAI_API_KEY" ]; then
    echo "⚠️  WARNING: OPENAI_API_KEY not set!"
    echo "   LLM features will not work properly"
    echo "   Set with: export OPENAI_API_KEY='your-key-here'"
    echo ""
    echo "Continue anyway? (y/n)"
    read -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Source ROS2
echo "Sourcing ROS2 environment..."
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Terminal 1: AirSim ROS Wrapper
echo ""
echo "[1/10] Starting AirSim ROS Wrapper..."
gnome-terminal --title="AirSim ROS" -- bash -c "source /home/mbs/Desktop/airsim/Cosys-AirSim/ros2/install/setup.bash && ros2 launch airsim_ros_pkgs airsim_node.launch.py; exec bash"
echo "      Waiting 7 seconds for AirSim initialization..."
sleep 7

# Terminal 2: MAVROS
echo "[2/10] Starting MAVROS..."
gnome-terminal --title="MAVROS" -- bash -c "ros2 run mavros mavros_node --ros-args --param fcu_url:='udp://:14550@127.0.0.1:14540'; exec bash"
echo "      Waiting 5 seconds for MAVROS connection..."
sleep 5

# Terminal 3: Velocity Coordinator (Critical - start early)
echo "[3/10] Starting Velocity Coordinator..."
gnome-terminal --title="Velocity Coordinator" -- bash -c "ros2 run search_patterns velocity_coordinator; exec bash"
echo "      Waiting 3 seconds for coordinator setup..."
sleep 3

# Terminal 4: Smart Keep-Alive Node
echo "[4/10] Starting Smart Keep-Alive Node..."
gnome-terminal --title="Keep-Alive" -- bash -c "ros2 run llm_controller smart_keep_alive_node; exec bash"
echo "      Waiting 3 seconds for keep-alive initialization..."
sleep 3

# Terminal 5: Vision Pipeline (YOLO Detection)
echo "[5/10] Starting Vision System (YOLO)..."
gnome-terminal --title="Vision System" -- bash -c "ros2 launch drone_vision_interpreter vision_pipeline.launch.py camera_topic:=/airsim_node/PX4/front_center_Scene/image; exec bash"
echo "      Waiting 5 seconds for vision system startup..."
sleep 5

# Terminal 6: Return to Home Node
echo "[6/10] Starting Return to Home (RTH) Node..."
gnome-terminal --title="RTH" -- bash -c "ros2 run search_patterns return_to_home; exec bash"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 7: Grid Search Node
echo "[7/10] Starting Grid Search Node..."
gnome-terminal --title="Grid Search" -- bash -c "ros2 run search_patterns fixed_grid_search; exec bash"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 8: Web Platform
echo "[8/10] Starting Web Platform..."
gnome-terminal --title="Web Platform" -- bash -c "ros2 run sar_web_platform web_server; exec bash"
echo "      Waiting 3 seconds for web server startup..."
sleep 3

# Terminal 9: Enhanced LLM Controller
echo "[9/10] Starting Enhanced LLM Controller..."
gnome-terminal --title="LLM Controller" -- bash -c "ros2 run llm_controller enhanced_llm; exec bash"
echo "      Waiting 3 seconds for LLM initialization..."
sleep 3

# Terminal 10: Status Monitor (Optional but helpful)
echo "[10/10] Starting System Status Monitor..."
gnome-terminal --title="Status Monitor" -- bash -c "watch -n 1 'echo \"=== SYSTEM STATUS ===\"; echo \"\"; echo \"Active Topics:\"; ros2 topic list | grep -E \"(command|detected|mavros/state)\"; echo \"\"; echo \"Node Status:\"; ros2 node list'"; exec bash"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║           SYSTEM STARTUP COMPLETE! 🚁                ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Available Features:"
echo "  ✓ Manual control via commands"
echo "  ✓ Web interface at http://localhost:5000"
echo "  ✓ YOLO object detection"
echo "  ✓ Grid search patterns"
echo "  ✓ Return to Home (RTH)"
echo "  ✓ Natural language control (LLM)"
echo ""
echo "═══ QUICK START COMMANDS ═══"
echo ""
echo "1. ARM & TAKEOFF:"
echo "   ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{custom_mode: 'OFFBOARD'}\""
echo "   ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\""
echo "   ros2 topic pub /simple_command std_msgs/msg/String \"data: 'up'\" --once"
echo ""
echo "2. MANUAL CONTROL:"
echo "   ros2 topic pub /simple_command std_msgs/msg/String \"data: 'forward/backward/left/right/up/down/stop'\" --once"
echo "   ros2 topic pub /simple_command std_msgs/msg/String \"data: 'yaw_left/yaw_right'\" --once"
echo ""
echo "3. AUTONOMOUS FEATURES:"
echo "   Search: ros2 topic pub /search_command std_msgs/msg/String \"data: 'search'\" --once"
echo "   RTH:    ros2 topic pub /rth_command std_msgs/msg/String \"data: 'rth'\" --once"
echo ""
echo "4. NATURAL LANGUAGE (LLM):"
echo "   ros2 topic pub /llm/command_input std_msgs/msg/String \"data: 'Find 3 people and hover when found'\" --once"
echo ""
echo "5. EMERGENCY:"
echo "   ros2 topic pub /emergency_stop std_msgs/msg/Bool \"data: true\" --once"
echo ""
echo "═══ MONITORING ═══"
echo ""
echo "Vision detections: ros2 topic echo /vision_detections"
echo "Drone state:       ros2 topic echo /mavros/state"
echo "Current position:  ros2 topic echo /mavros/local_position/pose"
echo ""
echo "All systems operational. Happy flying! 🚁"