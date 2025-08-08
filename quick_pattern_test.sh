#!/bin/bash
# Quick test to verify pattern system is working

echo "Quick Pattern System Test"
echo "========================"
echo ""
echo "This will generate a small expanding square pattern"
echo "and execute it for 10 seconds as a quick test."
echo ""
echo "Prerequisites:"
echo "- System running (./launch_hybrid_system.sh)"
echo "- Pattern nodes running (./test_patterns_standalone.sh)"
echo "- Drone armed and hovering"
echo ""
echo "Press Enter to start test..."
read

# Generate small pattern
echo "Generating 10m expanding square..."
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:10,2'" --once
sleep 2

# Start execution
echo "Starting pattern execution..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once

# Monitor for 10 seconds
echo "Running for 10 seconds..."
sleep 10

# Stop
echo "Stopping pattern..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once

echo ""
echo "Test complete! If the drone moved in a square pattern, the system is working."
echo "Run ./test_pattern_demo.sh for full demonstration."