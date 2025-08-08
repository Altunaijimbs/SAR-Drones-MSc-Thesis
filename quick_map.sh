#!/bin/bash
# Ultra-quick map launcher - one command to see drone position

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo "🗺️  DRONE POSITION MAP"
echo "═══════════════════════"
echo ""

# Just launch the most reliable visualizer
ros2 run search_patterns simple_grid_visualizer 2>/dev/null || \
ros2 run search_patterns background_visualizer 2>/dev/null || \
ros2 run search_patterns pattern_visualizer