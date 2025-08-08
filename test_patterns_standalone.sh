#!/bin/bash
# Test script for standalone pattern system
# This can be tested without affecting the working system

echo "╔══════════════════════════════════════════════════════╗"
echo "║    STANDALONE PATTERN SYSTEM TEST                    ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This tests the new pattern generator and executor"
echo "WITHOUT modifying any existing working components"
echo ""
echo "Prerequisites:"
echo "  - System launched with ./launch_hybrid_system.sh"
echo "  - Drone armed and hovering"
echo ""
echo "Press Enter to start pattern nodes..."
read

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Terminal 1: Pattern Generator
echo ""
echo "Starting Pattern Generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "ros2 run search_patterns pattern_generator; exec bash"
sleep 2

# Terminal 2: Pattern Executor
echo "Starting Pattern Executor..."
gnome-terminal --title="Pattern Executor" -- bash -c "ros2 run search_patterns pattern_executor; exec bash"
sleep 2

echo ""
echo "═══ PATTERN TEST COMMANDS ═══"
echo ""
echo "1. Generate expanding square pattern:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'expanding_square:10,3'\" --once"
echo ""
echo "2. Generate spiral pattern:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'spiral:20,5'\" --once"
echo ""
echo "3. Generate zigzag pattern:"
echo "   ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'zigzag:20,20,5'\" --once"
echo ""
echo "4. Start pattern execution:"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo ""
echo "5. Control commands:"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'pause'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'resume'\" --once"
echo "   ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'stop'\" --once"
echo ""
echo "6. Monitor pattern status:"
echo "   ros2 topic echo /pattern_status"
echo ""
echo "7. View generated waypoints:"
echo "   ros2 topic echo /pattern_waypoints"
echo ""
echo "NOTE: The pattern executor uses /position_command which works"
echo "      with our existing hybrid position controller"
echo ""