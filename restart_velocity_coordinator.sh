#!/bin/bash
# Restart velocity coordinator with updated pattern support

echo "Restarting velocity coordinator with pattern support..."

# Kill existing velocity coordinator
pkill -f velocity_coordinator
sleep 1

# Source and restart
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
ros2 run search_patterns velocity_coordinator &

sleep 2
echo ""
echo "Velocity coordinator restarted!"
echo ""
echo "Check it's working:"
echo "  ros2 node list | grep velocity"
echo ""
echo "Check active source:"
echo "  ros2 topic echo /velocity_coordinator/active_source"
echo ""
echo "Now the pattern executor should work!"