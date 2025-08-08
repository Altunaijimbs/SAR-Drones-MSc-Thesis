#!/bin/bash
# Test coordinated patterns - works WITH hybrid system, no conflicts!

echo "╔══════════════════════════════════════════════════════╗"
echo "║    COORDINATED PATTERN EXECUTION                     ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This works WITH launch_hybrid_system.sh!"
echo "  ✓ Uses velocity coordinator (priority 2)"
echo "  ✓ No control conflicts"
echo "  ✓ Rotates using simple commands"
echo "  ✓ Moves using position commands"
echo ""
echo "Prerequisites:"
echo "  - launch_hybrid_system.sh running"
echo "  - Drone armed and hovering"
echo ""
echo "Press Enter to start coordinated pattern nodes..."
read

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Terminal 1: Pattern Generator
echo "[1/3] Starting Pattern Generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns optimized_pattern_generator; exec bash"
sleep 2

# Terminal 2: Coordinated Pattern Executor
echo "[2/3] Starting Coordinated Pattern Executor..."
gnome-terminal --title="Coordinated Executor" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns coordinated_pattern_executor; exec bash"
sleep 2

# Terminal 3: Visualizer
echo "[3/3] Starting Visualizer..."
gnome-terminal --title="Grid Plot" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns simple_grid_visualizer; exec bash"
sleep 2

echo ""
echo "═══ PATTERN COMMANDS ═══"
echo ""
echo "1. ▶ Square Pattern:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'square:10'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "2. ▶ Expanding Square:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'expanding_square:8,3'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "3. ▶ Zigzag:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'zigzag:20,20,5'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "═══ HOW IT WORKS ═══"
echo ""
echo "The coordinated executor:"
echo "  • Publishes to /velocity_command_2 (priority 2)"
echo "  • Uses /simple_command for yaw rotation"
echo "  • Uses /position_command for movement"
echo "  • Works WITH velocity coordinator - no conflicts!"
echo ""
echo "Monitor velocity coordinator:"
echo "  ros2 topic echo /velocity_coordinator/active_source"
echo ""