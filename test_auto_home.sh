#!/bin/bash

echo "=== AUTO HOME SAVE TEST ==="
echo ""
echo "This tests if home position is automatically saved"
echo ""

echo "Step 1: Monitor RTH node output"
echo "In another terminal, run:"
echo "  ros2 run search_patterns return_to_home"
echo ""
echo "Look for these messages:"
echo "  - 'State changed - Armed: True, Mode: OFFBOARD'"
echo "  - 'Conditions met for auto-home save...'"
echo "  - 'HOME POSITION SAVED: (X, Y, Z)'"
echo ""
echo "Press Enter when monitoring terminal is ready..."
read

echo ""
echo "Step 2: Arm drone (should see state change)"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
sleep 1
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

echo ""
echo "Step 3: Take off above 2 meters"
echo "Auto-save should trigger when altitude > 2m"
ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once

echo ""
echo "Waiting 5 seconds for altitude..."
sleep 5

echo "Stopping ascent..."
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once

echo ""
echo "CHECK THE RTH NODE OUTPUT!"
echo "You should see:"
echo "1. 'Conditions met for auto-home save. Altitude: X.XXm, waiting 2s...'"
echo "2. After 2 seconds: 'HOME POSITION SAVED: (X, Y, Z)'"
echo ""
echo "If you don't see these messages, check:"
echo "- Is altitude > 2m? Check with: ros2 topic echo /mavros/local_position/pose --once"
echo "- Is mode OFFBOARD? Check with: ros2 topic echo /mavros/state --once"
echo ""
echo "To manually set home if auto-save failed:"
echo "  ros2 topic pub /set_home_position std_msgs/msg/String 'data: set' --once"