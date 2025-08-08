#!/bin/bash
# Advanced SAR Drone System with Custom Pattern Support
# This script launches all nodes including the advanced LLM controller

echo "╔══════════════════════════════════════════════════════╗"
echo "║   ADVANCED SAR DRONE SYSTEM - CUSTOM PATTERNS        ║"
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

# First build the workspace to get the new advanced_llm node
echo ""
echo "Building workspace to include new advanced LLM controller..."
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
colcon build --packages-select llm_controller
source install/setup.bash
cd -

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

# Terminal 7: Grid Search Node (Keep for fallback)
echo "[7/10] Starting Grid Search Node..."
gnome-terminal --title="Grid Search" -- bash -c "ros2 run search_patterns fixed_grid_search; exec bash"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 8: Web Platform
echo "[8/10] Starting Web Platform..."
gnome-terminal --title="Web Platform" -- bash -c "ros2 run sar_web_platform web_server; exec bash"
echo "      Waiting 3 seconds for web server startup..."
sleep 3

# Terminal 9: Advanced LLM Controller (NEW!)
echo "[9/10] Starting Advanced LLM Controller with Custom Patterns..."
gnome-terminal --title="Advanced LLM" -- bash -c "ros2 run llm_controller advanced_llm; exec bash"
echo "      Waiting 3 seconds for LLM initialization..."
sleep 3

# Terminal 10: Status Monitor
echo "[10/10] Starting System Status Monitor..."
gnome-terminal --title="Status Monitor" -- bash -c "watch -n 1 'echo \"=== SYSTEM STATUS ===\"; echo \"\"; echo \"Active Topics:\"; ros2 topic list | grep -E \"(command|detected|mavros/state)\"; echo \"\"; echo \"Node Status:\"; ros2 node list'"; exec bash"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║        ADVANCED SYSTEM STARTUP COMPLETE! 🚁          ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "🆕 NEW FEATURES:"
echo "  ✓ Custom search patterns via natural language"
echo "  ✓ Expanding square searches"
echo "  ✓ Spiral patterns"
echo "  ✓ Circle patterns"
echo "  ✓ Zigzag patterns"
echo "  ✓ Custom waypoint missions"
echo ""
echo "═══ EXAMPLE CUSTOM PATTERN COMMANDS ═══"
echo ""
echo "1. EXPANDING SQUARE:"
echo "   \"Do an expanding square search starting at 10 meters\""
echo "   \"Search in expanding squares, start small and grow by 15 meters each time\""
echo ""
echo "2. SPIRAL SEARCH:"
echo "   \"Do a spiral search pattern with 30 meter radius\""
echo "   \"Search in a spiral pattern outward from my current position\""
echo ""
echo "3. CIRCLE PATTERN:"
echo "   \"Fly in a circle with 20 meter radius\""
echo "   \"Do a circular search pattern around this area\""
echo ""
echo "4. ZIGZAG PATTERN:"
echo "   \"Do a zigzag search pattern 30 meters wide\""
echo "   \"Search back and forth in a zigzag pattern\""
echo ""
echo "═══ STANDARD COMMANDS ═══"
echo ""
echo "1. ARM & TAKEOFF:"
echo "   ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{custom_mode: 'OFFBOARD'}\""
echo "   ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\""
echo "   ros2 topic pub /simple_command std_msgs/msg/String \"data: 'up'\" --once"
echo ""
echo "2. NATURAL LANGUAGE (via terminal):"
echo "   ros2 topic pub /llm/command_input std_msgs/msg/String \"data: 'Your command here'\" --once"
echo ""
echo "3. WEB INTERFACE:"
echo "   Open http://localhost:5000 in your browser"
echo "   Use the Natural Language Control input field"
echo ""
echo "Happy flying with custom patterns! 🚁"