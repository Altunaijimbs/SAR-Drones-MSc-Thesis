#!/bin/bash
# Test FAST Precision Pattern Executor - Speeds doubled!

echo "╔══════════════════════════════════════════════════════╗"
echo "║    FAST PRECISION PATTERN EXECUTOR                   ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "⚡ SPEED IMPROVEMENTS:"
echo "  ✓ Cruise speed: 5.0 m/s (was 2.5)"
echo "  ✓ Turn speed: 2.0 m/s (was 0.5)"
echo "  ✓ No stopping at turns (continuous flow)"
echo "  ✓ Sharp turn threshold: 60° (was 45°)"
echo ""
echo "Works WITH launch_hybrid_system.sh!"
echo ""
echo "Prerequisites:"
echo "  - launch_hybrid_system.sh running"
echo "  - Drone armed and hovering"
echo ""
echo "Press Enter to start fast precision pattern system..."
read

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Terminal 1: Pattern Generator
echo "[1/3] Starting Pattern Generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns optimized_pattern_generator; exec bash"
sleep 2

# Terminal 2: Fast Precision Executor
echo "[2/3] Starting Fast Precision Executor..."
gnome-terminal --title="Fast Precision" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns precision_pattern_executor; exec bash"
sleep 2

# Terminal 3: Visualizer
echo "[3/3] Starting Grid Visualizer..."
gnome-terminal --title="Grid Plot" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns simple_grid_visualizer; exec bash"
sleep 2

echo ""
echo "═══ FAST PATTERN COMMANDS ═══"
echo ""
echo "1. Square (20m at 5 m/s):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'square:20'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "2. Zigzag (fast turns at 2 m/s):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'zigzag:20,20,5'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "3. Large Spiral:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'spiral:30,8'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "The drone should now fly MUCH faster while maintaining smooth turns!"