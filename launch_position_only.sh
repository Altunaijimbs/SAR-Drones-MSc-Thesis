#!/bin/bash
# Position-Only Control System (No Velocity Controllers)
# This prevents jittering from competing controllers

echo "╔══════════════════════════════════════════════════════╗"
echo "║   POSITION-ONLY CONTROL SYSTEM (NO JITTER)          ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This launch configuration uses ONLY position control"
echo "No velocity-based controllers to prevent conflicts"
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
echo "[1/9] Starting AirSim ROS Wrapper..."
gnome-terminal --title="AirSim ROS" -- bash -c "source /home/mbs/Desktop/airsim/Cosys-AirSim/ros2/install/setup.bash && ros2 launch airsim_ros_pkgs airsim_node.launch.py; exec bash"
echo "      Waiting 7 seconds for AirSim initialization..."
sleep 7

# Terminal 2: MAVROS
echo "[2/9] Starting MAVROS..."
gnome-terminal --title="MAVROS" -- bash -c "ros2 run mavros mavros_node --ros-args --param fcu_url:='udp://:14550@127.0.0.1:14540'; exec bash"
echo "      Waiting 5 seconds for MAVROS connection..."
sleep 5

# Terminal 3: Smooth Position Controller - THE ONLY MOVEMENT CONTROLLER
echo "[3/9] Starting Smooth Position Controller (No Oscillation)..."
gnome-terminal --title="Smooth Position Controller" -- bash -c "ros2 run search_patterns smooth_position_controller; exec bash"
echo "      Waiting 3 seconds for position controller..."
sleep 3

# SKIP velocity coordinator and smart_keep_alive to prevent conflicts!

# Terminal 4: Vision Pipeline (YOLO Detection)
echo "[4/9] Starting Vision System (YOLO)..."
gnome-terminal --title="Vision System" -- bash -c "ros2 launch drone_vision_interpreter vision_pipeline.launch.py camera_topic:=/airsim_node/PX4/front_center_Scene/image; exec bash"
echo "      Waiting 5 seconds for vision system startup..."
sleep 5

# Terminal 5: Return to Home Node (Modified to use position control)
echo "[5/9] Starting Return to Home (RTH) Node..."
gnome-terminal --title="RTH" -- bash -c "ros2 run search_patterns return_to_home; exec bash"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 6: Grid Search Node
echo "[6/9] Starting Grid Search Node..."
gnome-terminal --title="Grid Search" -- bash -c "ros2 run search_patterns fixed_grid_search; exec bash"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 7: Web Platform
echo "[7/9] Starting Web Platform..."
gnome-terminal --title="Web Platform" -- bash -c "ros2 run sar_web_platform web_server; exec bash"
echo "      Waiting 3 seconds for web server startup..."
sleep 3

# Terminal 8: Practical LLM Controller
echo "[8/9] Starting Practical LLM Controller..."
gnome-terminal --title="Practical LLM" -- bash -c "ros2 run llm_controller practical_llm; exec bash"
echo "      Waiting 3 seconds for LLM initialization..."
sleep 3

# Terminal 9: Status Monitor
echo "[9/9] Starting System Status Monitor..."
gnome-terminal --title="Status Monitor" -- bash -c "watch -n 1 'echo \"=== POSITION-ONLY SYSTEM STATUS ===\"; echo \"\"; echo \"Position Topics:\"; ros2 topic list | grep -E \"(position_command|mavros/setpoint_position)\"; echo \"\"; echo \"NO VELOCITY CONTROLLERS RUNNING (Prevents Jitter)\"; echo \"\"; echo \"Active Nodes:\"; ros2 node list | grep -E \"(position_controller|practical_llm|mavros)\"'"; exec bash"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║    POSITION-ONLY SYSTEM READY (NO JITTER) 🚁        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "⚠️  IMPORTANT NOTES:"
echo "  - NO velocity controllers running (prevents conflicts)"
echo "  - Position controller handles ALL movements"
echo "  - Manual velocity commands won't work"
echo "  - Use position commands or natural language only"
echo ""
echo "═══ MOVEMENT COMMANDS ═══"
echo ""
echo "1. ARM & TAKEOFF (Position-based):"
echo "   ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{custom_mode: 'OFFBOARD'}\""
echo "   ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\""
echo "   ros2 topic pub /position_command std_msgs/msg/String \"data: 'up:5'\" --once"
echo ""
echo "2. POSITION COMMANDS:"
echo "   ros2 topic pub /position_command std_msgs/msg/String \"data: 'forward:10'\" --once"
echo "   ros2 topic pub /position_command std_msgs/msg/String \"data: 'goto:5,5,5'\" --once"
echo "   ros2 topic pub /position_command std_msgs/msg/String \"data: 'hover'\" --once"
echo ""
echo "3. NATURAL LANGUAGE:"
echo "   Web: http://localhost:5000"
echo "   Terminal: ros2 topic pub /llm/command_input std_msgs/msg/String \"data: 'move forward 9 meters'\" --once"
echo ""
echo "Smooth flight without jitter! 🚁"