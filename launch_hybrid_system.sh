#!/bin/bash
# Hybrid Control System - Position commands via velocity control
# This prevents oscillation by using velocity commands to reach positions

echo "╔══════════════════════════════════════════════════════╗"
echo "║      HYBRID CONTROL SYSTEM (NO OSCILLATION)          ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This system uses:"
echo "  - Hybrid position controller (converts position → velocity)"
echo "  - Velocity coordinator (manages priorities)"
echo "  - Smart keep-alive (for manual control)"
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

# Terminal 3: Velocity Coordinator (manages all velocity sources)
echo "[3/11] Starting Velocity Coordinator..."
gnome-terminal --title="Velocity Coordinator" -- bash -c "ros2 run search_patterns velocity_coordinator; exec bash"
echo "      Waiting 3 seconds for coordinator setup..."
sleep 3

# Terminal 4: Smart Keep-Alive Node (for manual control)
echo "[4/11] Starting Smart Keep-Alive Node..."
gnome-terminal --title="Keep-Alive" -- bash -c "ros2 run llm_controller smart_keep_alive_node; exec bash"
echo "      Waiting 3 seconds for keep-alive initialization..."
sleep 3

# Terminal 5: Hybrid Position Controller (NEW!)
echo "[5/11] Starting Hybrid Position Controller..."
gnome-terminal --title="Hybrid Position" -- bash -c "ros2 run search_patterns hybrid_position_controller; exec bash"
echo "      Waiting 3 seconds for hybrid controller..."
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

# Terminal 10: Practical LLM Controller
echo "[10/11] Starting Practical LLM Controller..."
gnome-terminal --title="Practical LLM" -- bash -c "ros2 run llm_controller practical_llm; exec bash"
echo "      Waiting 3 seconds for LLM initialization..."
sleep 3

# Terminal 11: Status Monitor
echo "[11/11] Starting System Status Monitor..."
gnome-terminal --title="Status Monitor" -- bash -c "watch -n 1 'echo \"=== HYBRID SYSTEM STATUS ===\"; echo \"\"; echo \"Velocity Sources:\"; ros2 topic list | grep -E \"(velocity_command|position_command)\"; echo \"\"; echo \"Current velocity source:\"; ros2 topic echo /velocity_coordinator/active_source --once 2>/dev/null || echo \"Waiting...\"; echo \"\"; echo \"Key Nodes:\"; ros2 node list | grep -E \"(hybrid|velocity_coordinator|practical_llm)\"'"; exec bash"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║        HYBRID SYSTEM READY (NO OSCILLATION) 🚁       ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "✅ KEY FEATURES:"
echo "  - Position commands converted to smooth velocity control"
echo "  - Works with existing velocity coordinator"
echo "  - No position control oscillation"
echo "  - All movement types supported"
echo ""
echo "═══ MOVEMENT COMMANDS ═══"
echo ""
echo "1. ARM & TAKEOFF:"
echo "   ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{custom_mode: 'OFFBOARD'}\""
echo "   ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\""
echo "   ros2 topic pub /simple_command std_msgs/msg/String \"data: 'up'\" --once"
echo ""
echo "2. POSITION COMMANDS (smooth, no oscillation):"
echo "   ros2 topic pub /position_command std_msgs/msg/String \"data: 'forward:10'\" --once"
echo "   ros2 topic pub /position_command std_msgs/msg/String \"data: 'right:3'\" --once"
echo "   ros2 topic pub /position_command std_msgs/msg/String \"data: 'goto:5,5,5'\" --once"
echo ""
echo "3. MANUAL VELOCITY (still works):"
echo "   ros2 topic pub /simple_command std_msgs/msg/String \"data: 'forward'\" --once"
echo "   ros2 topic pub /simple_command std_msgs/msg/String \"data: 'stop'\" --once"
echo ""
echo "4. NATURAL LANGUAGE:"
echo "   Web: http://localhost:5000"
echo "   Terminal: ros2 topic pub /llm/command_input std_msgs/msg/String \"data: 'move forward 9 meters'\" --once"
echo ""
echo "The hybrid controller will smoothly fly to positions without oscillation!"