#!/bin/bash
# Test attitude-based pattern execution - proper yaw, pitch, roll control

echo "╔══════════════════════════════════════════════════════╗"
echo "║    ATTITUDE-BASED PATTERN CONTROL                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This system uses PROPER flight control:"
echo "  ✓ Drone rotates to face direction FIRST"
echo "  ✓ Then moves forward (like real flying)"
echo "  ✓ No more sliding sideways to waypoints"
echo "  ✓ Natural pitch/roll movements"
echo "  ✓ Speed control: cruise & approach speeds"
echo ""
echo "Prerequisites:"
echo "  - System running (./launch_hybrid_system.sh)"
echo "  - Drone armed and hovering"
echo ""
echo "Press Enter to start attitude-based pattern system..."
read

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Terminal 1: Pattern Generator (can use any - optimized is good)
echo "[1/3] Starting Pattern Generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns optimized_pattern_generator; exec bash"
sleep 2

# Terminal 2: Attitude Pattern Executor (NEW!)
echo "[2/3] Starting Attitude Pattern Executor..."
gnome-terminal --title="Attitude Executor" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns attitude_pattern_executor; exec bash"
sleep 2

# Terminal 3: Visualizer
echo "[3/3] Starting Visualizer..."
gnome-terminal --title="Grid Plot" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns simple_grid_visualizer; exec bash"
sleep 2

echo ""
echo "═══ PATTERN COMMANDS (with attitude control) ═══"
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
echo "═══ WHAT TO EXPECT ═══"
echo ""
echo "The drone will:"
echo "  1. ROTATE to face each waypoint"
echo "  2. Move FORWARD (not sideways)"
echo "  3. Slow down when approaching waypoints"
echo "  4. Make proper turns at corners"
echo ""
echo "Control modes shown in status:"
echo "  • 'rotating' - Turning to face target"
echo "  • 'moving' - Flying at cruise speed"
echo "  • 'approaching' - Slowing down near waypoint"
echo "  • 'idle' - Hovering"
echo ""
echo "Monitor status:"
echo "  ros2 topic echo /pattern_status"
echo ""
echo "Monitor MAVROS setpoint_raw:"
echo "  ros2 topic echo /mavros/setpoint_raw/local"
echo ""