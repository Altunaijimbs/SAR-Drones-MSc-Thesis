#!/bin/bash

echo "RTH Direction Testing - Bypassing RTH Node"
echo "=========================================="
echo ""
echo "This will test different coordinate transformations"
echo "WITHOUT using the RTH node to avoid hijacking"
echo ""

echo "Step 1: Kill any existing RTH node"
pkill -f return_to_home
echo "RTH node killed"
echo ""

echo "Step 2: Source ROS2"
echo "cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws"
echo "source install/setup.bash"
echo ""

echo "Step 3: Run the test node"
echo "python3 /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/src/search_patterns/search_patterns/test_rth_directions.py"
echo ""

echo "Step 4: In another terminal, fly the drone away from home:"
echo "ros2 topic pub /simple_command std_msgs/msg/String \"data: 'forward'\" --once"
echo "# Wait 5 seconds"
echo "ros2 topic pub /simple_command std_msgs/msg/String \"data: 'right'\" --once"
echo "# Wait 5 seconds"
echo "ros2 topic pub /simple_command std_msgs/msg/String \"data: 'stop'\" --once"
echo ""

echo "Step 5: Test different transformations:"
echo ""
echo "Test 1 - Original (no negation):"
echo "ros2 topic pub /test_rth_cmd std_msgs/msg/String \"data: 'test_original'\" --once"
echo "# Observe direction for 5 seconds"
echo ""
echo "Stop:"
echo "ros2 topic pub /test_rth_cmd std_msgs/msg/String \"data: 'stop'\" --once"
echo ""

echo "Test 2 - Both negated (current attempt):"
echo "ros2 topic pub /test_rth_cmd std_msgs/msg/String \"data: 'test_negated'\" --once"
echo "# Observe direction for 5 seconds"
echo ""

echo "Test 3 - No transformation:"
echo "ros2 topic pub /test_rth_cmd std_msgs/msg/String \"data: 'test_no_transform'\" --once"
echo "# Observe direction for 5 seconds"
echo ""

echo "Test 4 - Only X negated:"
echo "ros2 topic pub /test_rth_cmd std_msgs/msg/String \"data: 'test_single_neg_x'\" --once"
echo "# Observe direction for 5 seconds"
echo ""

echo "Test 5 - Only Y negated:"
echo "ros2 topic pub /test_rth_cmd std_msgs/msg/String \"data: 'test_single_neg_y'\" --once"
echo "# Observe direction for 5 seconds"
echo ""

echo "The test that makes the drone move FORWARD and LEFT toward home is the correct transformation!"