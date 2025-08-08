#!/bin/bash
# Debug script to test if visualizer is receiving drone position

echo "╔══════════════════════════════════════════════════════╗"
echo "║        PATTERN VISUALIZER DEBUG TEST                 ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This will help debug why the map isn't updating"
echo ""

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo "═══ CHECKING DRONE POSITION TOPIC ═══"
echo ""
echo "Checking if drone position is being published..."
timeout 2 ros2 topic echo /mavros/local_position/pose --once

if [ $? -eq 0 ]; then
    echo "✅ Drone position is being published!"
else
    echo "❌ No drone position data found!"
    echo "   Make sure MAVROS is running and connected to PX4"
    exit 1
fi

echo ""
echo "═══ TESTING VISUALIZER ═══"
echo ""
echo "Starting Pattern Visualizer..."
echo "The map window should appear and show:"
echo "  • Red dot at current drone position"
echo "  • Blue line as drone moves"
echo ""
echo "Try moving the drone with:"
echo "  ros2 topic pub /simple_command std_msgs/msg/String \"data: 'forward'\" --once"
echo "  ros2 topic pub /simple_command std_msgs/msg/String \"data: 'right'\" --once"
echo ""
echo "If the red dot doesn't move, try:"
echo "1. Click on the map window to make it active"
echo "2. Check the terminal for any error messages"
echo ""

# Start visualizer
ros2 run search_patterns pattern_visualizer