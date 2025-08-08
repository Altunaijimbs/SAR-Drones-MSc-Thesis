#!/bin/bash
# Debug pattern execution system

echo "╔══════════════════════════════════════════════════════╗"
echo "║    PATTERN SYSTEM DEBUGGER                           ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# Monitor velocity commands
echo "1. Monitoring /velocity_command_2 (pattern executor output):"
gnome-terminal --title="Pattern Velocity Output" -- bash -c "ros2 topic echo /velocity_command_2" &

echo ""
echo "2. Monitoring /velocity_coordinator/active_source:"
gnome-terminal --title="Active Source" -- bash -c "ros2 topic echo /velocity_coordinator/active_source" &

echo ""
echo "3. Monitoring /mavros/setpoint_velocity/cmd_vel_unstamped (final output):"
gnome-terminal --title="MAVROS Velocity" -- bash -c "ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped" &

echo ""
echo "4. Monitoring /pattern_status:"
gnome-terminal --title="Pattern Status" -- bash -c "ros2 topic echo /pattern_status" &

echo ""
echo "5. Monitoring /velocity_priority_2:"
gnome-terminal --title="Priority" -- bash -c "ros2 topic echo /velocity_priority_2" &

echo ""
echo "═══ QUICK CHECKS ═══"
echo ""
echo "Check if velocity coordinator is running:"
echo "  ros2 node list | grep velocity_coordinator"
echo ""
echo "List all velocity topics:"
echo "  ros2 topic list | grep velocity"
echo ""
echo "Check pattern executor is publishing:"
echo "  ros2 topic hz /velocity_command_2"
echo ""
echo "═══ RESTART VELOCITY COORDINATOR ═══"
echo ""
echo "If needed, restart velocity coordinator:"
echo "  pkill -f velocity_coordinator"
echo "  ros2 run search_patterns velocity_coordinator &"
echo ""