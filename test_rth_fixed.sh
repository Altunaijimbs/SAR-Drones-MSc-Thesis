#!/bin/bash

echo "=== RTH TEST PROCEDURE ==="
echo "This will test if Return to Home is working correctly"
echo ""
echo "STEP 1: Launch the system"
echo "In a new terminal, run:"
echo "  cd /home/mbs/SAR-Drones-MSc-Thesis"
echo "  ./launch_with_vision.sh"
echo ""
echo "Press Enter when system is running..."
read

echo ""
echo "STEP 2: Check topics are active"
ros2 topic list | grep -E "(mavros|rth|velocity)" | head -10
echo ""
echo "Press Enter to continue..."
read

echo ""
echo "STEP 3: Arm and takeoff"
echo "Running arming sequence..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
sleep 1
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
sleep 1

echo "Taking off to 5 meters..."
ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once
sleep 5
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once

echo ""
echo "STEP 4: Move drone away from home position"
echo "Moving forward 10 meters..."
ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once
sleep 5

echo "Moving right 10 meters..."
ros2 topic pub /simple_command std_msgs/msg/String "data: 'right'" --once
sleep 5

echo "Stopping..."
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once
sleep 2

echo ""
echo "STEP 5: Check current position"
echo "Current position should be approximately (10, 10, 5) from home"
echo "Press Enter to activate RTH..."
read

echo ""
echo "STEP 6: ACTIVATING RTH"
echo "Watch the drone - it should fly DIRECTLY toward home, not backward!"
ros2 topic pub /rth_command std_msgs/msg/String "data: 'rth'" --once

echo ""
echo "STEP 7: Monitor RTH progress"
echo "In another terminal, you can run:"
echo "  ros2 topic echo /velocity_coordinator/active_source"
echo "  (should show 'Active: rth')"
echo ""
echo "To see velocities being sent:"
echo "  ros2 topic echo /rth/velocity_command"
echo ""
echo "Press Enter to test RTH cancellation..."
read

echo ""
echo "STEP 8: Test RTH cancellation"
echo "Sending manual command to cancel RTH..."
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once

echo ""
echo "RTH should now be cancelled. The drone should stop."
echo "You can verify by checking velocity coordinator - it should no longer show 'rth'"
echo ""
echo "=== TEST COMPLETE ==="
echo ""
echo "EXPECTED BEHAVIOR:"
echo "✓ Drone flies FORWARD and LEFT toward home (not backward)"
echo "✓ RTH can be cancelled with any manual command"
echo "✓ Drone hovers in place after RTH is cancelled"
echo ""
echo "DEBUGGING COMMANDS:"
echo "- Check home position: ros2 topic echo /mavros/local_position/pose --once"
echo "- Monitor RTH debug: ros2 run search_patterns return_to_home (look for [RTH DEBUG] messages)"
echo "- Emergency stop: ros2 topic pub /emergency_stop std_msgs/msg/Bool 'data: true' --once"