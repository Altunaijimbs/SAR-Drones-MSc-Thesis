#!/bin/bash
# Test script for pattern LLM bridge with visualization
# Tests the bridge between LLM commands and pattern system with real-time map

echo "╔══════════════════════════════════════════════════════╗"
echo "║  PATTERN LLM BRIDGE TEST WITH VISUALIZATION          ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This tests the pattern_llm_bridge with real-time map"
echo ""
echo "The map will display:"
echo "  • Blue line: Actual drone path"
echo "  • Green dashed line: Planned waypoints"
echo "  • Red dot: Current drone position"
echo "  • Yellow dot: Current target waypoint"
echo ""
echo "Prerequisites:"
echo "  - System launched with ./launch_hybrid_system.sh"
echo "  - Drone armed and hovering"
echo ""
echo "Press Enter to start pattern nodes, bridge, and visualizer..."
read

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Terminal 1: Pattern Visualizer (Map)
echo ""
echo "[1/4] Starting Pattern Visualizer (Map window will appear)..."
gnome-terminal --title="Pattern Map" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns pattern_visualizer; exec bash"
sleep 3

# Terminal 2: Pattern Generator
echo "[2/4] Starting Pattern Generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns pattern_generator; exec bash"
sleep 2

# Terminal 3: Pattern Executor
echo "[3/4] Starting Pattern Executor..."
gnome-terminal --title="Pattern Executor" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns pattern_executor; exec bash"
sleep 2

# Terminal 4: Pattern LLM Bridge
echo "[4/4] Starting Pattern LLM Bridge..."
gnome-terminal --title="Pattern LLM Bridge" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run llm_controller pattern_llm_bridge; exec bash"
sleep 2

echo ""
echo "═══ PATTERN LLM BRIDGE TEST COMMANDS ═══"
echo ""
echo "Test the bridge with these LLM-style commands:"
echo "(Watch the map window to see the drone move!)"
echo ""
echo "1. ▶ Start expanding square search:"
echo "   ros2 topic pub /llm/pattern_request std_msgs/msg/String \"data: 'search:expanding_square'\" --once"
echo ""
echo "2. ▶ Start spiral search:"
echo "   ros2 topic pub /llm/pattern_request std_msgs/msg/String \"data: 'search:spiral'\" --once"
echo ""
echo "3. ▶ Start zigzag search:"
echo "   ros2 topic pub /llm/pattern_request std_msgs/msg/String \"data: 'search:zigzag'\" --once"
echo ""
echo "4. ▶ Stop pattern:"
echo "   ros2 topic pub /llm/pattern_request std_msgs/msg/String \"data: 'stop_pattern'\" --once"
echo ""
echo "5. ▶ Pause pattern:"
echo "   ros2 topic pub /llm/pattern_request std_msgs/msg/String \"data: 'pause_pattern'\" --once"
echo ""
echo "6. ▶ Resume pattern:"
echo "   ros2 topic pub /llm/pattern_request std_msgs/msg/String \"data: 'resume_pattern'\" --once"
echo ""
echo "═══ MONITORING ═══"
echo ""
echo "Watch what happens:"
echo "  1. LLM command → /llm/pattern_request"
echo "  2. Bridge translates → /pattern_command & /pattern_control"
echo "  3. Pattern executor → /position_command"
echo "  4. Map shows drone movement in real-time!"
echo ""
echo "Monitor topics:"
echo "  ros2 topic echo /pattern_status"
echo "  ros2 topic echo /pattern_waypoints"
echo ""
echo "Default parameters:"
echo "  • Expanding square: 15m sides, 4 loops"
echo "  • Spiral: 25m radius, 5m spacing"
echo "  • Zigzag: 30x30m area, 5m spacing"
echo ""