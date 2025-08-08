#!/bin/bash
# Monitor velocity coordinator and hybrid position controller

echo "═══ VELOCITY SYSTEM MONITOR ═══"
echo ""
echo "This monitors the velocity coordination system"
echo "Press Ctrl+C to exit"
echo ""

# Function to show velocity sources
show_sources() {
    echo -e "\n[VELOCITY SOURCES]"
    echo "Priority 0: /keepalive/velocity_command (hover)"
    echo "Priority 1: /llm/velocity_command (manual)"
    echo "Priority 2: /hybrid_position/velocity_command (position control)"
    echo "Priority 3: /mavros/setpoint_velocity/cmd_vel_unstamped_safe (avoidance)"
    echo "Priority 4: /search_pattern/velocity_command (search)"
    echo "Priority 5: /rth/velocity_command (RTH)"
}

# Show sources once
show_sources

echo -e "\n[MONITORING ACTIVE SOURCE AND COMMANDS]"
echo "---"

# Monitor in a loop
while true; do
    # Get active source
    echo -n "Active: "
    ros2 topic echo /velocity_coordinator/active_source --once 2>/dev/null | grep "data:" | cut -d' ' -f2 || echo "none"
    
    # Get current velocity being sent to MAVROS
    echo -n "Velocity to MAVROS: "
    timeout 0.5 ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped --once 2>/dev/null | grep -A3 "linear:" | grep -E "x:|y:|z:" | tr '\n' ' ' || echo "waiting..."
    
    echo "---"
    sleep 1
done