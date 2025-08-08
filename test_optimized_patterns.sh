#!/bin/bash
# Test optimized patterns - fixes overshooting and rotation issues

echo "╔══════════════════════════════════════════════════════╗"
echo "║      OPTIMIZED PATTERN SYSTEM (NO OVERSHOOT)         ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Fixes:"
echo "  ✓ No more circular loops at corners"
echo "  ✓ Automatic stop after pattern completion"
echo "  ✓ Approach waypoints before sharp turns"
echo "  ✓ Speed reduction near waypoints"
echo "  ✓ Stabilization at corners"
echo ""
echo "Prerequisites:"
echo "  - System running (./launch_hybrid_system.sh)"
echo "  - Drone armed and hovering"
echo ""
echo "Press Enter to start optimized pattern nodes..."
read

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Terminal 1: Optimized Pattern Generator
echo "[1/3] Starting Optimized Pattern Generator..."
gnome-terminal --title="Optimized Pattern Generator" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns optimized_pattern_generator; exec bash"
sleep 2

# Terminal 2: Optimized Pattern Executor
echo "[2/3] Starting Optimized Pattern Executor..."
gnome-terminal --title="Optimized Pattern Executor" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns optimized_pattern_executor; exec bash"
sleep 2

# Terminal 3: Grid Plot Visualizer
echo "[3/3] Starting Grid Plot Visualizer..."
gnome-terminal --title="Grid Plot" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns simple_grid_visualizer; exec bash"
sleep 2

echo ""
echo "═══ OPTIMIZED PATTERN COMMANDS ═══"
echo ""
echo "1. ▶ Clean Square (with approach waypoints):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'square:10'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "2. ▶ Expanding Square (no overlap):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'expanding_square:8,3'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "3. ▶ Smooth Zigzag (gradual turns):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'zigzag:20,20,5'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "4. ▶ Smooth Spiral:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'spiral:15,4'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "═══ CONTROL COMMANDS ═══"
echo ""
echo "Pause:  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'pause'\" --once"
echo "Resume: ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'resume'\" --once"
echo "Stop:   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'stop'\" --once"
echo ""
echo "═══ KEY IMPROVEMENTS ═══"
echo ""
echo "• Automatic stop when pattern completes (no more rotation!)"
echo "• Approach waypoints reduce overshoot at corners"
echo "• Speed reduction and stabilization at sharp turns"
echo "• Cleaner paths without circular loops"
echo ""
echo "Monitor status:"
echo "  ros2 topic echo /pattern_status"
echo ""
echo "Status will show 'completed': true when done"
echo ""