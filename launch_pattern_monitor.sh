#!/bin/bash
# Launch terminal-based pattern monitor

echo "Starting Pattern Monitor (Terminal-based)..."
echo ""
echo "This shows pattern execution status without GUI dependencies"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
ros2 run search_patterns pattern_monitor