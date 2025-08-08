#!/bin/bash
# Test precision pattern executor - No jittering, no overshooting!

echo "╔══════════════════════════════════════════════════════╗"
echo "║    PRECISION PATTERN EXECUTOR                        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Key Features:"
echo "  ✓ Works WITH velocity coordinator (no jittering!)"
echo "  ✓ Sharp turn detection and handling"
echo "  ✓ Automatic speed reduction at turns"
echo "  ✓ Approach/departure waypoints for smooth turns"
echo "  ✓ Priority system integration"
echo ""
echo "IMPORTANT: Run WITH launch_hybrid_system.sh!"
echo "Prerequisites:"
echo "  - launch_hybrid_system.sh MUST be running"
echo "  - Drone armed and hovering"
echo ""
echo "Press Enter to build and start precision pattern nodes..."
read

# Build
echo "Building system..."
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
colcon build --packages-select search_patterns
source install/setup.bash

# Terminal 1: Pattern Generator
echo "[1/3] Starting Optimized Pattern Generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns optimized_pattern_generator; exec bash"
sleep 2

# Terminal 2: Precision Pattern Executor
echo "[2/3] Starting Precision Pattern Executor..."
gnome-terminal --title="Precision Executor" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns precision_pattern_executor; exec bash"
sleep 2

# Terminal 3: Visualizer
echo "[3/3] Starting Grid Visualizer..."
gnome-terminal --title="Grid Plot" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns simple_grid_visualizer; exec bash"
sleep 2

echo ""
echo "═══ TEST SEQUENCE ═══"
echo ""
echo "1. ▶ Test Square (should have smooth corners):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'square:8'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "2. ▶ Test Zigzag (should handle sharp turns):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'zigzag:10,10,3'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "3. ▶ Test Spiral (smooth curves):"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'spiral:15,4'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "═══ MONITOR ═══"
echo ""
echo "Check velocity coordinator (should show priority 2 when running):"
echo "  ros2 topic echo /velocity_coordinator/active_source"
echo ""
echo "Watch pattern status:"
echo "  ros2 topic echo /pattern_status"
echo ""
echo "═══ KEY IMPROVEMENTS ═══"
echo ""
echo "1. NO JITTERING: Works WITH velocity coordinator using priority 2"
echo "2. SHARP TURN HANDLING: Detects turns >45° and adds approach points"
echo "3. SPEED CONTROL: Automatically slows down for turns"
echo "4. SMOOTH TRANSITIONS: Gradual acceleration/deceleration"
echo ""
echo "The zigzag pattern should now:"
echo "  • Slow down before sharp turns"
echo "  • Stop briefly at turn points"
echo "  • Accelerate smoothly after turns"
echo "  • No overshooting!"