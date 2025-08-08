#!/bin/bash
# Build script for SAR Drone System

echo "╔══════════════════════════════════════════════════════╗"
echo "║         SAR DRONE SYSTEM - BUILD SCRIPT              ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# Navigate to workspace
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws

echo "Building ROS2 packages..."
echo ""

# Full build (uncomment if needed)
# colcon build

# Build only the packages we've modified
echo "Building llm_controller package..."
colcon build --packages-select llm_controller

echo ""
echo "Building search_patterns package..."
colcon build --packages-select search_patterns

echo ""
echo "Building sar_web_platform package..."
colcon build --packages-select sar_web_platform

echo ""
echo "Sourcing workspace..."
source install/setup.bash

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║              BUILD COMPLETE! ✓                       ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Verifying new nodes are available:"
echo ""
echo "Position Controller:"
ros2 pkg executables search_patterns | grep position_controller
echo ""
echo "LLM Controllers:"
ros2 pkg executables llm_controller | grep -E "(practical|advanced|enhanced)"
echo ""
echo "You can now run:"
echo "  ./launch_practical_system.sh"
echo "  ./launch_complete_system.sh"
echo ""