#!/bin/bash
# Test improved patterns with better waypoints and yaw control

echo "╔══════════════════════════════════════════════════════╗"
echo "║      IMPROVED PATTERN SYSTEM TEST                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Improvements:"
echo "  ✓ Square pattern without overlapping paths"
echo "  ✓ Zigzag with sharp 90° turns"
echo "  ✓ Yaw control - drone faces direction of travel"
echo "  ✓ New lawnmower pattern option"
echo ""
echo "Prerequisites:"
echo "  - System running (./launch_hybrid_system.sh)"
echo "  - Drone armed and hovering"
echo ""
echo "Press Enter to start improved pattern nodes..."
read

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Terminal 1: Improved Pattern Generator
echo "[1/3] Starting Improved Pattern Generator..."
gnome-terminal --title="Improved Pattern Generator" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns improved_pattern_generator; exec bash"
sleep 2

# Terminal 2: Improved Pattern Executor
echo "[2/3] Starting Improved Pattern Executor..."
gnome-terminal --title="Improved Pattern Executor" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns improved_pattern_executor; exec bash"
sleep 2

# Terminal 3: Terminal Map for visualization
echo "[3/4] Starting Terminal Map..."
gnome-terminal --title="Terminal Map" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns terminal_map; exec bash"
sleep 2

# Terminal 4: Simple Grid Visualizer (matplotlib plot)
echo "[4/4] Starting Grid Plot Visualizer..."
gnome-terminal --title="Grid Plot" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns simple_grid_visualizer; exec bash"
sleep 2

echo ""
echo "═══ IMPROVED PATTERN COMMANDS ═══"
echo ""
echo "1. ▶ Clean Expanding Square (no overlap):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'expanding_square:8,3'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "2. ▶ Sharp Zigzag (90° turns):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'zigzag:20,20,5'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "3. ▶ Spiral with Yaw Control:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'spiral:15,4'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "4. ▶ NEW - Lawnmower Pattern:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'lawnmower:20,20,5'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "═══ CONTROL COMMANDS ═══"
echo ""
echo "Pause:  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'pause'\" --once"
echo "Resume: ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'resume'\" --once"
echo "Stop:   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'stop'\" --once"
echo ""
echo "═══ MONITORING ═══"
echo ""
echo "ros2 topic echo /pattern_status"
echo "ros2 topic echo /pattern_waypoints"
echo ""
echo "The Terminal Map will show the drone movement with ASCII graphics."
echo "Watch for:"
echo "  • Cleaner square patterns (no overlap)"
echo "  • Sharp 90° turns in zigzag"
echo "  • Drone rotating to face travel direction"
echo ""