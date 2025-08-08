#!/bin/bash
# Pattern test with real-time map visualization

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     PATTERN TEST WITH REAL-TIME MAP VISUALIZATION          ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "This will show a live map of the drone's movement!"
echo ""
echo "The map will display:"
echo "  • Blue line: Actual drone path"
echo "  • Green dashed line: Planned waypoints"
echo "  • Red dot: Current drone position"
echo "  • Yellow dot: Current target waypoint"
echo ""
echo "Prerequisites:"
echo "- System running (./launch_hybrid_system.sh)"
echo "- Drone armed and hovering"
echo ""
echo "Press Enter to start..."
read

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Start Pattern Visualizer (Map)
echo "Starting Pattern Visualizer (Map window will appear)..."
gnome-terminal --title="Pattern Map" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns pattern_visualizer; exec bash"
sleep 3

# Start Pattern Generator
echo "Starting Pattern Generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns pattern_generator; exec bash"
sleep 2

# Start Pattern Executor
echo "Starting Pattern Executor..."
gnome-terminal --title="Pattern Executor" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns pattern_executor; exec bash"
sleep 2

echo ""
echo "═══ READY FOR PATTERN TESTING ═══"
echo ""
echo "Example commands to try:"
echo ""
echo "1. Small square pattern:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'expanding_square:8,2'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "2. Small spiral pattern:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'spiral:12,4'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "3. Zigzag pattern:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'zigzag:15,15,5'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "Control commands:"
echo "   Pause:  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'pause'\" --once"
echo "   Resume: ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'resume'\" --once"
echo "   Stop:   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'stop'\" --once"
echo ""
echo "The map window will show the drone's movement in real-time!"
echo ""