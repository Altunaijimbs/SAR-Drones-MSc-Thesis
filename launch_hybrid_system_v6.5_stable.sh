#!/bin/bash
# Hybrid Control System V6 THESIS - TRUE LLM-Based Autonomous SAR
# MSc Thesis Demonstration: Natural Language → Complex SAR Behaviors

echo "╔══════════════════════════════════════════════════════╗"
echo "║  V6.5 STABLE - SIMPLIFIED SAR WITH GPS LOGGING 🎯    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "🎓 MSc THESIS DEMONSTRATION SYSTEM"
echo "   Natural Language Understanding → Autonomous SAR Behaviors"
echo ""
echo "NEW IN V6.5 - STABLE & SIMPLIFIED:"
echo "  🧠 Same TRUE LLM Controller (OpenAI GPT)"
echo "  🚁 Simplified SAR: Approach → Center → Log GPS → Resume"
echo "  📝 Natural language: 'Do a zigzag search and find people'"
echo "  🔄 15 seconds per survivor (was 60+ with orbiting)"
echo "  📍 GPS coordinate logging to CSV file"
echo ""
echo "SIMPLIFIED SAR BEHAVIOR:"
echo "  1. Execute search pattern at altitude (20m)"
echo "  2. Detect person → confirm with 5 detections"
echo "  3. Approach to 10m distance"
echo "  4. Center target in camera view"
echo "  5. Log GPS coordinates to CSV"
echo "  6. Resume search pattern immediately"
echo ""
echo "⚠️  CRITICAL REQUIREMENT FOR THESIS:"
echo "   OPENAI_API_KEY MUST be set for LLM-based control!"
echo ""

# MANDATORY: Check for OpenAI API key
if [ -z "$OPENAI_API_KEY" ]; then
    echo "❌ ERROR: OPENAI_API_KEY not set!"
    echo ""
    echo "This MSc thesis is about LLM-BASED drone control."
    echo "OpenAI API is ESSENTIAL, not optional!"
    echo ""
    echo "Set with: export OPENAI_API_KEY='your-key-here'"
    echo ""
    echo "Cannot proceed without LLM capability for thesis demonstration."
    exit 1
fi

echo "✅ OpenAI API key detected - LLM capability enabled"
echo ""
echo "Prerequisites:"
echo "  ✓ PX4 is running and waiting for simulator"
echo "  ✓ UE5 is playing with cameras configured"
echo "  ✓ Internet connection for OpenAI API calls"
echo ""
echo "Press Enter when ready to start..."
read

# Source ROS2
echo "Sourcing ROS2 environment..."
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Kill any existing nodes from previous sessions
echo "Cleaning up previous sessions..."
pkill -f "pattern_executor.py"
pkill -f "pattern_generator.py"
pkill -f "updated_vf_sharpy"
pkill -f "simple_grid_visualizer"
pkill -f "velocity_coordinator"
pkill -f "llm_controller"
pkill -f "sar_executor"
pkill -f "velocity_router"
sleep 2

# Terminal 1: AirSim ROS Wrapper
echo ""
echo "[1/18] Starting AirSim ROS Wrapper..."
gnome-terminal --title="AirSim ROS" -- bash -c "source /home/mbs/Desktop/airsim/Cosys-AirSim/ros2/install/setup.bash && ros2 launch airsim_ros_pkgs airsim_node.launch.py; exec bash"
echo "      Waiting 7 seconds for AirSim initialization..."
sleep 7

# Terminal 2: MAVROS
echo "[2/18] Starting MAVROS..."
gnome-terminal --title="MAVROS" -- bash -c "ros2 run mavros mavros_node --ros-args --param fcu_url:='udp://:14550@127.0.0.1:14540'; exec bash"
echo "      Waiting 5 seconds for MAVROS connection..."
sleep 5

# Terminal 3: Velocity Coordinator V3
echo "[3/18] Starting Velocity Coordinator V3..."
gnome-terminal --title="Velocity Coordinator V3" -- bash -c "ros2 run search_patterns improved_velocity_coordinator; exec bash"
echo "      ✅ Using V3 coordinator with priority system"
echo "      Waiting 3 seconds for coordinator setup..."
sleep 3

# Terminal 4: Smart Keep-Alive Node
echo "[4/18] Starting Smart Keep-Alive Node..."
gnome-terminal --title="Keep-Alive" -- bash -c "ros2 run llm_controller smart_keep_alive_node; exec bash"
echo "      Waiting 3 seconds for keep-alive initialization..."
sleep 3

# Terminal 5: Hybrid Position Controller
echo "[5/18] Starting Hybrid Position Controller..."
gnome-terminal --title="Hybrid Position" -- bash -c "ros2 run search_patterns hybrid_position_controller; exec bash"
echo "      Waiting 3 seconds for hybrid controller..."
sleep 3

# Terminal 6: Vision Pipeline
echo "[6/18] Starting Vision System - YOLO..."
gnome-terminal --title="Vision System" -- bash -c "ros2 launch drone_vision_interpreter vision_pipeline.launch.py camera_topic:=/airsim_node/PX4/front_center_Scene/image; exec bash"
echo "      🎯 Vision system for person detection"
echo "      Waiting 5 seconds for vision system startup..."
sleep 5

# Terminal 7: Return to Home Node
echo "[7/18] Starting Return to Home - RTH Node..."
gnome-terminal --title="RTH" -- bash -c "ros2 run search_patterns return_to_home; exec bash"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 8: Grid Search Node
echo "[8/18] Starting Grid Search Node..."
gnome-terminal --title="Grid Search" -- bash -c "ros2 run search_patterns fixed_grid_search; exec bash"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 9: Enhanced Web Platform
echo "[9/18] Starting Enhanced Web Platform..."
gnome-terminal --title="Web Platform" -- bash -c "cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws && source install/setup.bash && python3 /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/src/sar_web_platform/sar_web_platform/enhanced_web_server.py; exec bash"
echo "      Waiting 3 seconds for web server startup..."
sleep 3

# Terminal 10: Pattern Generator
echo "[10/18] Starting Pattern Generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "ros2 run search_patterns pattern_generator; exec bash"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 11: Pattern Executor
echo "[11/18] Starting VF Sharpy Tuned Pattern Executor..."
gnome-terminal --title="Pattern Executor" -- bash -c "ros2 run search_patterns updated_vf_sharpy_tuned; exec bash"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 12: Enhanced Map Visualizer (with person tracking!)
echo "[12/18] Starting Enhanced Map Visualizer..."
gnome-terminal --title="Enhanced Map" -- bash -c "ros2 run search_patterns enhanced_map_visualizer; exec bash"
echo "      🗺️ Shows drone path AND detected person location!"
echo "      📏 Real-time distance estimation visualization"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 13: 🧠 THESIS LLM CONTROLLER (V6 - TRUE LLM!)
echo "[13/18] 🧠 Starting THESIS LLM Controller (TRUE AI UNDERSTANDING)..."
gnome-terminal --title="Thesis LLM" -- bash -c "ros2 run llm_controller thesis_llm_controller; exec bash"
echo "      ✨ Uses OpenAI GPT as PRIMARY method"
echo "      🎓 Demonstrates natural language understanding for thesis"
echo "      📝 Understands: 'Do a zigzag search and find people'"
echo "      Waiting 3 seconds for LLM initialization..."
sleep 3

# Terminal 14: 🚁 V6.5 AUTONOMOUS SAR EXECUTOR - SIMPLIFIED
echo "[14/18] 🚁 Starting V6.5 SAR Executor (Simplified: Center & Log GPS)..."
gnome-terminal --title="V6.5 SAR Executor" -- bash -c "ros2 run llm_controller autonomous_sar_executor_v6_5; exec bash"
echo "      📏 Distance estimation methods:"
echo "         • Depth camera (most accurate)"
echo "         • Bounding box size estimation"
echo "         • Altitude-based estimation"
echo "      📋 V6.5 behavior: Search → Detect → Approach → Center → Log GPS → Resume"
echo "      Waiting 3 seconds for SAR executor..."
sleep 3

# Terminal 15: Vision Detection Bridge (Fixed)
echo "[15/18] Starting Vision Detection Bridge..."
gnome-terminal --title="Vision Bridge" -- bash -c "ros2 run llm_controller vision_detection_bridge_fixed; exec bash"
echo "      Converts SceneDescription to detection format"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 16: Autonomous Velocity Router
echo "[16/18] Starting Autonomous Velocity Router..."
gnome-terminal --title="Velocity Router" -- bash -c "ros2 run search_patterns autonomous_velocity_router; exec bash"
echo "      Routes autonomous velocity commands"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 17: Autonomous Position Router
echo "[17/18] Starting Autonomous Position Router..."
gnome-terminal --title="Position Router" -- bash -c "ros2 run search_patterns autonomous_position_router; exec bash"
echo "      Routes autonomous position commands"
echo "      Waiting 2 seconds..."
sleep 2

# Terminal 18: V6 Thesis Status Monitor
echo "[18/18] Starting V6 Thesis Status Monitor..."
gnome-terminal --title="V6 Status" -- bash -c "watch -n 1 'echo \"╔══════════════════════════════════════════════════════╗\"; echo \"║     V6 THESIS - LLM-BASED SAR STATUS                ║\"; echo \"╚══════════════════════════════════════════════════════╝\"; echo \"\"; echo \"🧠 LLM INTERPRETATION:\"; ros2 topic echo /llm/interpretation --once 2>/dev/null | python3 -c \"import sys, json; data=sys.stdin.read(); j=json.loads(data.split(\\\"data: \\\\\\\"\\\")[1].split(\\\"\\\\\\\"\\\")[0]) if \\\"data:\" in data else {}; print(f\\\"  Original: {j.get(\\\"original\\\", \\\"N/A\\\")}\\\"); print(f\\\"  Understanding: {j.get(\\\"interpretation\\\", \\\"N/A\\\")}\\\"); print(f\\\"  LLM Time: {j.get(\\\"llm_time\\\", 0):.2f}s\\\")\" 2>/dev/null || echo \"  No LLM commands yet\"; echo \"\"; echo \"🚁 SAR STATE:\"; ros2 topic echo /sar/status --once 2>/dev/null | python3 -c \"import sys, json; data=sys.stdin.read(); j=json.loads(data.split(\\\"data: \\\\\\\"\\\")[1].split(\\\"\\\\\\\"\\\")[0]) if \\\"data:\" in data else {}; print(f\\\"  State: {j.get(\\\"state\\\", \\\"idle\\\")}\\\"); print(f\\\"  Message: {j.get(\\\"message\\\", \\\"No mission\\\")}\\\"); print(f\\\"  Victim Detected: {j.get(\\\"victim_detected\\\", False)}\\\"); print(f\\\"  Altitude: {j.get(\\\"current_altitude\\\", 0):.1f}m\\\"); print(f\\\"  Pattern: {j.get(\\\"pattern\\\", \\\"none\\\")}\\\")\" 2>/dev/null || echo \"  No active mission\"; echo \"\"; echo \"👁️ VISION:\"; ros2 topic echo /drone/scene_description --once 2>/dev/null | grep -E \"person|people\" | head -2 || echo \"  No person detected\"; echo \"\"; echo \"📊 PATTERN:\"; ros2 topic echo /pattern_status --once 2>/dev/null | head -2 || echo \"  No pattern active\"'; exec bash"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  V6.5 STABLE SYSTEM READY - GPS LOGGING! 🎯          ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "🎓 MSc THESIS DEMONSTRATION:"
echo ""
echo "This system demonstrates LLM-based autonomous SAR with:"
echo "  • Natural language understanding via OpenAI GPT"
echo "  • Complex behavior sequences from simple commands"
echo "  • Intelligent mission execution with state management"
echo ""
echo "═══ NATURAL LANGUAGE COMMANDS (via LLM) ═══"
echo ""
echo "Try these natural language commands:"
echo ""
echo "COMPLEX SAR MISSIONS:"
echo "  ros2 topic pub /llm/command_input std_msgs/msg/String \\"
echo "    \"data: 'Do a zigzag search and find people'\" --once"
echo ""
echo "  ros2 topic pub /llm/command_input std_msgs/msg/String \\"
echo "    \"data: 'Search for survivors using a spiral pattern'\" --once"
echo ""
echo "  ros2 topic pub /llm/command_input std_msgs/msg/String \\"
echo "    \"data: 'Look for injured people with expanding squares'\" --once"
echo ""
echo "SIMPLE MOVEMENTS (also via LLM):"
echo "  ros2 topic pub /llm/command_input std_msgs/msg/String \\"
echo "    \"data: 'Move ahead slowly'\" --once"
echo ""
echo "  ros2 topic pub /llm/command_input std_msgs/msg/String \\"
echo "    \"data: 'Go up and rotate left'\" --once"
echo ""
echo "═══ EXPECTED BEHAVIOR ═══"
echo ""
echo "When you say 'Do a zigzag search and find people':"
echo ""
echo "1. LLM understands: zigzag pattern + person search"
echo "2. Starts zigzag pattern at current altitude"
echo "3. If person detected:"
echo "   • Saves current waypoint"
echo "   • Descends to 10m"
echo "   • Approaches to 5m from victim"
echo "   • Orbits 2x while facing victim"
echo "   • Returns to original altitude"
echo "   • Resumes pattern from saved waypoint"
echo ""
echo "═══ THESIS KEY POINTS ═══"
echo ""
echo "✅ TRUE AI understanding (not keyword matching)"
echo "✅ Complex autonomous behaviors from natural language"
echo "✅ State management and mission resumption"
echo "✅ Integration of vision, planning, and control"
echo ""
echo "Web Interface: http://localhost:5000"
echo ""
echo "Monitor LLM interpretation in the V6 Status terminal!"
echo ""
echo "Press Ctrl+C in this terminal to stop all nodes."

# Wait for user to stop
wait