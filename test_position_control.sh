#!/bin/bash
# Test script for position-based control

echo "╔══════════════════════════════════════════════════════╗"
echo "║        POSITION CONTROL TEST SEQUENCE                ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This script demonstrates position-based movement control"
echo ""
echo "Prerequisites:"
echo "  - System launched with ./launch_practical_system.sh"
echo "  - Drone armed and in OFFBOARD mode"
echo "  - Drone already taken off"
echo ""
echo "Press Enter to start test sequence..."
read

echo ""
echo "═══ TEST 1: Basic Movements ═══"
echo ""

echo "1. Moving forward 5 meters..."
ros2 topic pub /position_command std_msgs/msg/String "data: 'forward:5'" --once
sleep 6

echo "2. Moving right 3 meters..."
ros2 topic pub /position_command std_msgs/msg/String "data: 'right:3'" --once
sleep 6

echo "3. Moving up 2 meters..."
ros2 topic pub /position_command std_msgs/msg/String "data: 'up:2'" --once
sleep 4

echo "4. Moving backward 5 meters..."
ros2 topic pub /position_command std_msgs/msg/String "data: 'backward:5'" --once
sleep 6

echo "5. Moving left 3 meters..."
ros2 topic pub /position_command std_msgs/msg/String "data: 'left:3'" --once
sleep 6

echo "6. Moving down 2 meters..."
ros2 topic pub /position_command std_msgs/msg/String "data: 'down:2'" --once
sleep 4

echo ""
echo "═══ TEST 2: Natural Language Commands ═══"
echo ""

echo "1. Testing: 'Go forward 10 meters'"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'Go forward 10 meters'" --once
sleep 12

echo "2. Testing: 'Move up 3 meters and then go right 5 meters'"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'Move up 3 meters'" --once
sleep 5
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'Now go right 5 meters'" --once
sleep 7

echo "3. Testing: 'Return to home'"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'Return to home base'" --once
sleep 10

echo ""
echo "═══ TEST 3: Search Pattern ═══"
echo ""

echo "Testing: 'Search for people in the area'"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'Search for people in the area and hover if you find someone'" --once

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║            TEST SEQUENCE COMPLETE! 🚁                ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Monitor the drone's behavior in UE5"
echo "Check the terminal windows for controller outputs"
echo ""