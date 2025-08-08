#!/bin/bash
# Simple test for pattern visualizer alone

echo "╔══════════════════════════════════════════════════════╗"
echo "║        SIMPLE VISUALIZER TEST                        ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Testing if the visualizer can track drone movement"
echo ""
echo "Prerequisites:"
echo "  - System running (./launch_hybrid_system.sh)"
echo "  - Drone armed and in the air"
echo ""

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo "Starting Pattern Visualizer..."
echo ""
echo "The map should show:"
echo "  • Red dot = current drone position"
echo "  • Blue line = path history as drone moves"
echo ""
echo "Move the drone with these commands in another terminal:"
echo ""
echo "  ros2 topic pub /simple_command std_msgs/msg/String \"data: 'forward'\" --once"
echo "  ros2 topic pub /simple_command std_msgs/msg/String \"data: 'backward'\" --once"
echo "  ros2 topic pub /simple_command std_msgs/msg/String \"data: 'left'\" --once"
echo "  ros2 topic pub /simple_command std_msgs/msg/String \"data: 'right'\" --once"
echo ""
echo "The red dot should move and leave a blue trail!"
echo ""

# Run the visualizer
ros2 run search_patterns pattern_visualizer