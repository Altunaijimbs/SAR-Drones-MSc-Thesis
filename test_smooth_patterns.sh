#!/bin/bash
# Test smooth trajectory patterns - AeroStack2 inspired approach

echo "╔══════════════════════════════════════════════════════╗"
echo "║    SMOOTH TRAJECTORY PATTERN EXECUTION               ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Features:"
echo "  ✓ Trapezoidal velocity profiles"
echo "  ✓ Acceleration planning"
echo "  ✓ Velocity feedforward"
echo "  ✓ No overshooting!"
echo ""
echo "Prerequisites:"
echo "  - Build the system first"
echo "  - launch_hybrid_system.sh running"
echo "  - Drone armed and hovering"
echo ""
echo "Press Enter to build and start smooth pattern nodes..."
read

# Build
echo "Building system..."
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
colcon build --packages-select search_patterns
source install/setup.bash

# Terminal 1: Pattern Generator
echo "[1/3] Starting Pattern Generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns optimized_pattern_generator; exec bash"
sleep 2

# Terminal 2: Smooth Trajectory Executor
echo "[2/3] Starting Smooth Trajectory Executor..."
gnome-terminal --title="Smooth Executor" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns smooth_trajectory_executor; exec bash"
sleep 2

# Terminal 3: Visualizer
echo "[3/3] Starting Grid Visualizer..."
gnome-terminal --title="Grid Plot" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns simple_grid_visualizer; exec bash"
sleep 2

echo ""
echo "═══ TEST COMMANDS ═══"
echo ""
echo "1. ▶ Small Square (5m):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'square:5'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "2. ▶ Medium Square (10m):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'square:10'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "3. ▶ Zigzag Pattern:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'zigzag:15,15,4'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "4. ⏸ Control Commands:"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'pause'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'resume'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'stop'\" --once"
echo ""
echo "═══ MONITOR ═══"
echo ""
echo "Watch status:"
echo "  ros2 topic echo /pattern_status"
echo ""
echo "Watch setpoints:"
echo "  ros2 topic echo /mavros/setpoint_raw/local"
echo ""
echo "This executor should eliminate overshooting through:"
echo "  • Trapezoidal velocity profiles"
echo "  • Smooth acceleration/deceleration"
echo "  • Velocity feedforward control"
echo "  • Tighter position thresholds"