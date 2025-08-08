#!/bin/bash
# Practical SAR Drone System with Position Control
# This launches the system with realistic capabilities

echo "╔══════════════════════════════════════════════════════╗"
echo "║   PRACTICAL SAR DRONE SYSTEM - POSITION CONTROL      ║"
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
echo "[1/11] Starting AirSim ROS Wrapper..."
gnome-terminal --title="AirSim ROS" -- bash -c "source /home/mbs/Desktop/airsim/Cosys-AirSim/ros2/install/setup.bash && ros2 launch airsim_ros_pkgs airsim_node.launch.py; exec bash"
echo "      Waiting 7 seconds for AirSim initialization..."
sleep 7

# Terminal 2: MAVROS
echo "[2/11] Starting MAVROS..."
gnome-terminal --title="MAVROS" -- bash -c "ros2 run mavros mavros_node --ros-args --param fcu_url:='udp://:14550@127.0.0.1:14540'; exec bash"
echo "      Waiting 5 seconds for MAVROS connection..."
sleep 5

# Terminal 3: Position Controller (NEW!)
echo "[3/11] Starting Position Controller..."
gnome-terminal --title="Position Controller" -- bash -c "ros2 run search_patterns position_controller; exec bash"
echo "      Waiting 3 seconds for position controller..."
sleep 3

# Terminal 4: Velocity Coordinator
echo "[4/11] Starting Velocity Coordinator..."
gnome-terminal --title="Velocity Coordinator" -- bash -c "ros2 run search_patterns velocity_coordinator; exec bash"
echo "      Waiting 3 seconds for coordinator setup..."
sleep 3

# Terminal 5: Smart Keep-Alive Node
echo "[5/11] Starting Smart Keep-Alive Node..."
gnome-terminal --title="Keep-Alive" -- bash -c "ros2 run llm_controller smart_keep_alive_node; exec bash"
echo "      Waiting 3 seconds for keep-alive initialization..."
sleep 3

# Terminal 6: Vision Pipeline (YOLO Detection)
echo "[6/11] Starting Vision System (YOLO)..."
gnome-terminal --title="Vision System" -- bash -c "ros2 launch drone_vision_interpreter vision_pipeline.launch.py camera_topic:=/airsim_node/PX4/front_center_Scene/image; exec bash"
echo "      Waiting 5 seconds for vision system startup..."
sleep 5

# Terminal 7: Return to Home Node
echo "[7/11] Starting Return to Home (RTH) Node..."
gnome-terminal --title="RTH" -- bash -c "ros2 run search_patterns return_to_home; exec bash"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 8: Grid Search Node
echo "[8/11] Starting Grid Search Node..."
gnome-terminal --title="Grid Search" -- bash -c "ros2 run search_patterns fixed_grid_search; exec bash"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 9: Web Platform
echo "[9/11] Starting Web Platform..."
gnome-terminal --title="Web Platform" -- bash -c "ros2 run sar_web_platform web_server; exec bash"
echo "      Waiting 3 seconds for web server startup..."
sleep 3

# Terminal 10: Practical LLM Controller (NEW!)
echo "[10/11] Starting Practical LLM Controller..."
gnome-terminal --title="Practical LLM" -- bash -c "ros2 run llm_controller practical_llm; exec bash"
echo "      Waiting 3 seconds for LLM initialization..."
sleep 3

# Terminal 11: Status Monitor
echo "[11/11] Starting System Status Monitor..."
gnome-terminal --title="Status Monitor" -- bash -c "watch -n 1 'echo \"=== SYSTEM STATUS ===\"; echo \"\"; echo \"Active Topics:\"; ros2 topic list | grep -E \"(position_command|simple_command|search_command|mavros/setpoint_position)\"; echo \"\"; echo \"Node Status:\"; ros2 node list | grep -E \"(position_controller|practical_llm|mavros)\"'"; exec bash"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║      PRACTICAL SYSTEM STARTUP COMPLETE! 🚁          ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "🆕 KEY FEATURES:"
echo "  ✓ Position-based control for precise movements"
echo "  ✓ Natural language understanding with realistic expectations"
echo "  ✓ Grid search pattern (pre-programmed)"
echo "  ✓ Return to Home (RTH)"
echo "  ✓ Vision-based detection with hovering"
echo ""
echo "═══ SUPPORTED NATURAL LANGUAGE COMMANDS ═══"
echo ""
echo "PRECISE MOVEMENT:"
echo "  \"Go forward 5 meters\""
echo "  \"Move backward 3 meters\""
echo "  \"Go up 2 meters\""
echo "  \"Move left 4 meters\""
echo ""
echo "ABSOLUTE POSITION:"
echo "  \"Go to position 10, 20, 5\""
echo "  \"Fly to coordinates x=15, y=25, z=10\""
echo ""
echo "SEARCH & RESCUE:"
echo "  \"Search for people\""
echo "  \"Do a grid search and hover when you find someone\""
echo "  \"Find missing hikers\""
echo ""
echo "NAVIGATION:"
echo "  \"Return to home\""
echo "  \"Come back to base\""
echo "  \"Stop and hover\""
echo ""
echo "═══ QUICK START ═══"
echo ""
echo "1. ARM & TAKEOFF:"
echo "   ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{custom_mode: 'OFFBOARD'}\""
echo "   ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\""
echo "   ros2 topic pub /position_command std_msgs/msg/String \"data: 'up:5'\" --once"
echo ""
echo "2. TEST POSITION CONTROL:"
echo "   ros2 topic pub /position_command std_msgs/msg/String \"data: 'forward:5'\" --once"
echo "   ros2 topic pub /position_command std_msgs/msg/String \"data: 'goto:10,10,5'\" --once"
echo ""
echo "3. NATURAL LANGUAGE (via Web or Terminal):"
echo "   Web: http://localhost:5000"
echo "   Terminal: ros2 topic pub /llm/command_input std_msgs/msg/String \"data: 'Go forward 10 meters'\" --once"
echo ""
echo "Happy flying with practical capabilities! 🚁"