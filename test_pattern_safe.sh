#!/bin/bash
# Safe pattern test with smaller, slower patterns

echo "╔════════════════════════════════════════════════════════════╗"
echo "║         SAFE PATTERN TEST (Anti-Oscillation)               ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "This test uses smaller patterns with proper delays"
echo "to prevent oscillation issues."
echo ""
echo "Prerequisites:"
echo "- System running (./launch_hybrid_system.sh)"
echo "- Drone armed and hovering at safe altitude (>5m)"
echo ""
echo "Press Enter to continue..."
read

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Test 1: Very small square
echo ""
echo "Test 1: Small 5m square pattern (single loop)"
echo "Generating pattern..."
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:5,1'" --once
sleep 2

echo "Starting execution..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once

echo "Running for 20 seconds..."
sleep 20

echo "Stopping..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
sleep 3

# Test 2: Small spiral
echo ""
echo "Test 2: Small spiral pattern (10m radius)"
echo "Press Enter to continue..."
read

echo "Generating pattern..."
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:10,3'" --once
sleep 2

echo "Starting execution..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once

echo "Running for 30 seconds..."
sleep 30

echo "Stopping..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once

echo ""
echo "Test complete!"
echo ""
echo "If oscillation persists, try these commands manually:"
echo "  - Increase position threshold: ros2 param set /pattern_executor position_threshold 3.0"
echo "  - Test single waypoint: ros2 topic pub /position_command std_msgs/msg/String \"data: 'goto:5,5,5'\" --once"
echo ""