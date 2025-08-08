#!/bin/bash
# Test AirSim Native Pattern Executor - Uses AirSim's built-in path following!

echo "╔══════════════════════════════════════════════════════╗"
echo "║    AIRSIM NATIVE PATTERN EXECUTOR                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "🚀 KEY FEATURES:"
echo "  ✓ Uses AirSim's moveOnPath for SMOOTH flight"
echo "  ✓ Dynamic lookahead (like path.py example)"
echo "  ✓ 5 m/s cruise speed (fast!)"
echo "  ✓ Natural banking on turns"
echo "  ✓ No overshooting - guaranteed!"
echo ""
echo "⚠️  IMPORTANT: This bypasses MAVROS/PX4 entirely!"
echo "   Works directly with AirSim API"
echo ""
echo "Prerequisites:"
echo "  - UE5 with AirSim running"
echo "  - Drone spawned in simulation"
echo ""
echo "Press Enter to build and start AirSim native pattern system..."
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

# Terminal 2: AirSim Native Executor
echo "[2/3] Starting AirSim Native Executor..."
gnome-terminal --title="AirSim Native" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns airsim_native_executor; exec bash"
sleep 2

# Terminal 3: Simple Grid Visualizer
echo "[3/3] Starting Grid Visualizer..."
gnome-terminal --title="Grid Plot" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns simple_grid_visualizer; exec bash"
sleep 2

echo ""
echo "═══ AIRSIM NATIVE TEST COMMANDS ═══"
echo ""
echo "1. 🔷 Square Pattern (smooth corners):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'square:20'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "2. 🔀 Zigzag Pattern (natural turns):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'zigzag:20,20,5'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "3. 🌀 Spiral Pattern (perfect curves):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'spiral:30,6'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "4. ⏸️ Control:"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'pause'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'resume'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'stop'\" --once"
echo ""
echo "═══ MONITOR ═══"
echo ""
echo "Watch pattern status:"
echo "  ros2 topic echo /pattern_status"
echo ""
echo "═══ HOW IT WORKS ═══"
echo ""
echo "This uses the SAME approach as Cosys AirSim's path.py:"
echo "  • moveOnPathAsync with all waypoints at once"
echo "  • Dynamic lookahead = velocity + velocity/2"
echo "  • ForwardOnly drivetrain (always faces forward)"
echo "  • Adaptive lookahead for smooth curves"
echo ""
echo "The result: Smooth, fast, realistic drone flight!"