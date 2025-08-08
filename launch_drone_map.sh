#!/bin/bash
# Launch just the drone movement map visualization

echo "╔════════════════════════════════════════════════════════════╗"
echo "║              DRONE MOVEMENT MAP VIEWER                     ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "This will display a real-time map of drone movement"
echo "Works with any movement - manual control, patterns, RTH, etc."
echo ""
echo "Starting map visualization..."

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
ros2 run search_patterns pattern_visualizer