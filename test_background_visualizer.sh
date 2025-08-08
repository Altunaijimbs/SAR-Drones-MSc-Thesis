#!/bin/bash
# Test the background visualizer - updates every 500ms without stealing focus

echo "╔══════════════════════════════════════════════════════╗"
echo "║        BACKGROUND VISUALIZER TEST                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This visualizer:"
echo "  • Updates every 500ms (not too aggressive)"
echo "  • Runs in background without stealing focus"
echo "  • Shows grid plot with drone movement"
echo "  • Uses matplotlib animation framework properly"
echo ""
echo "Prerequisites:"
echo "  - System running (./launch_hybrid_system.sh)"
echo "  - Drone can be on ground or already flying"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo "Starting Background Visualizer..."
echo "The plot window will appear but won't steal focus from your terminal"
echo ""
echo "Move the drone with:"
echo "  ros2 topic pub /simple_command std_msgs/msg/String \"data: 'forward'\" --once"
echo "  ros2 topic pub /simple_command std_msgs/msg/String \"data: 'right'\" --once"
echo ""

ros2 run search_patterns background_visualizer