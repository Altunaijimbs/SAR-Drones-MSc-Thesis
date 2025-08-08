#!/bin/bash
# Test script for LLM natural language movement commands

echo "╔══════════════════════════════════════════════════════╗"
echo "║      LLM MOVEMENT COMMAND TEST SEQUENCE              ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This script tests natural language movement commands"
echo ""
echo "Prerequisites:"
echo "  - System launched with ./launch_hybrid_system.sh"
echo "  - Drone armed and in OFFBOARD mode"
echo "  - Drone already taken off (hovering)"
echo ""
echo "Watch the following terminals:"
echo "  - Practical LLM terminal - for command parsing"
echo "  - Hybrid Position terminal - for movement execution"
echo "  - Velocity Coordinator - for priority management"
echo ""
echo "Press Enter to start test sequence..."
read

echo ""
echo "═══ TEST 1: Basic Forward Movement ═══"
echo ""

echo "1. Testing: 'move ahead 9 meters'"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'move ahead 9 meters'" --once
echo "   Waiting 12 seconds for movement..."
sleep 12

echo ""
echo "2. Testing: 'go forward 5 meters'"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'go forward 5 meters'" --once
echo "   Waiting 8 seconds..."
sleep 8

echo ""
echo "═══ TEST 2: Different Phrasings ═══"
echo ""

echo "3. Testing: 'Move forward 10 meters please'"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'Move forward 10 meters please'" --once
echo "   Waiting 12 seconds..."
sleep 12

echo ""
echo "4. Testing: 'fly backward 7 meters'"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'fly backward 7 meters'" --once
echo "   Waiting 10 seconds..."
sleep 10

echo ""
echo "═══ TEST 3: Other Directions (Including Small Movements) ═══"
echo ""

echo "5. Testing: 'move right 3 meters' (small movement test)"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'move right 3 meters'" --once
echo "   Waiting 5 seconds..."
sleep 5

echo ""
echo "6. Testing: 'go left 4 meters'"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'go left 4 meters'" --once
echo "   Waiting 6 seconds..."
sleep 6

echo ""
echo "7. Testing: 'ascend 3 meters'"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'ascend 3 meters'" --once
echo "   Waiting 5 seconds..."
sleep 5

echo ""
echo "8. Testing: 'descend 2 meters'"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'descend 2 meters'" --once
echo "   Waiting 4 seconds..."
sleep 4

echo ""
echo "═══ TEST 4: Complex Commands ═══"
echo ""

echo "9. Testing: 'hover at current position'"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'hover at current position'" --once
sleep 3

echo ""
echo "10. Testing absolute position: 'go to position 0, 0, 5'"
ros2 topic pub /llm/command_input std_msgs/msg/String "data: 'go to position 0, 0, 5'" --once
echo "    Waiting for return to origin..."
sleep 15

echo ""
echo "═══ MONITORING TIP ═══"
echo ""
echo "To monitor the velocity system in another terminal:"
echo "  ./monitor_velocity.sh"
echo ""
echo "This shows:"
echo "  - Active velocity source (should be 'hybrid' during movements)"
echo "  - Actual velocity being sent to drone"
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║         MOVEMENT TEST COMPLETE! 🚁                   ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Check for:"
echo "✓ Smooth movement without oscillation"
echo "✓ Small movements (like 'right 3 meters') work correctly"
echo "✓ Drone stops smoothly at target position"
echo "✓ No back-and-forth swinging"
echo ""
echo "If successful, we can proceed to search patterns!"
echo ""