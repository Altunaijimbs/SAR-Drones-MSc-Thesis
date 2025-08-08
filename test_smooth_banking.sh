#!/bin/bash
# Test the Smooth Banking Executor - simplified and stable

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     SMOOTH BANKING EXECUTOR TEST                          ║"
echo "║     Simplified for Stability - No Jittering               ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo "This executor features:"
echo "  ✓ Simple waypoint following"
echo "  ✓ Smooth yaw control (no aggressive turning)"
echo "  ✓ Stable velocity commands"
echo "  ✓ Turn-then-move strategy for sharp corners"
echo ""
echo "Press ENTER to start..."
read

# Clean up
pkill -f pattern_executor 2>/dev/null
pkill -f pattern_generator 2>/dev/null
sleep 2

# Launch pattern generator
echo "Starting Pattern Generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    ros2 run search_patterns pattern_generator
" &
GEN_PID=$!
sleep 2

# Launch smooth banking executor
echo "Starting Smooth Banking Executor..."
gnome-terminal --title="Smooth Banking Executor" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo 'Smooth Banking Executor'
    echo 'Stable control without jittering'
    ros2 run search_patterns smooth_banking_executor
" &
EXEC_PID=$!
sleep 2

# Test with small square first
echo ""
echo "Test 1: Small Square (5m)"
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:5,1'" --once
sleep 1
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once

echo "Testing for 30 seconds..."
sleep 30

ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
sleep 2

echo ""
echo "How was the performance?"
echo "1. No jittering (smooth)"
echo "2. Some jittering"
echo "3. Lots of jittering"
echo -n "Your answer (1-3): "
read performance

if [ "$performance" == "1" ]; then
    echo "Great! Testing larger pattern..."
    
    echo ""
    echo "Test 2: Larger Square (10m)"
    ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:10,1'" --once
    sleep 1
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
    
    sleep 40
    
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
fi

# Cleanup
kill $GEN_PID $EXEC_PID 2>/dev/null
pkill -f pattern_executor 2>/dev/null
pkill -f pattern_generator 2>/dev/null

echo ""
echo "Test complete!"
echo ""
echo "If still jittering, try:"
echo "  1. precision_pattern_executor (your previous best)"
echo "  2. improved_pattern_executor"
echo "  3. Adjust parameters in smooth_banking_executor.py"