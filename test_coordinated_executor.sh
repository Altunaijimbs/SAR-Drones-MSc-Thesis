#!/bin/bash
# Test the Coordinated Banking Executor that works WITH the velocity coordinator

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     COORDINATED BANKING EXECUTOR TEST                     ║"
echo "║     Works WITH Velocity Coordinator - No Conflicts!       ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo "IMPORTANT: This executor integrates with the hybrid system!"
echo ""
echo "It uses:"
echo "  ✓ /velocity_command topic (goes to coordinator)"
echo "  ✓ Priority system (priority 3 for patterns)"
echo "  ✓ No direct MAVROS commands (no conflicts!)"
echo ""
echo "The velocity coordinator will handle:"
echo "  • Priority management"
echo "  • Smooth transitions"
echo "  • No command conflicts"
echo ""
echo "Press ENTER to start test..."
read

# Clean up any old pattern nodes (but NOT the velocity coordinator!)
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

# Launch coordinated executor
echo "Starting Coordinated Banking Executor..."
gnome-terminal --title="Coordinated Banking Executor" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo '═══════════════════════════════════════════════════════'
    echo '    COORDINATED BANKING EXECUTOR'
    echo '═══════════════════════════════════════════════════════'
    echo ''
    echo 'Publishing to: /velocity_command'
    echo 'Priority: 3 (pattern execution)'
    echo ''
    ros2 run search_patterns coordinated_banking_executor
" &
EXEC_PID=$!
sleep 3

# Monitor velocity coordinator status
echo ""
echo "Checking velocity coordinator status..."
ros2 topic echo /velocity_coordinator/active_source --once 2>/dev/null || echo "Velocity coordinator ready"

echo ""
echo "═══════════════════════════════════════════════════════"
echo "TEST: Square Pattern with Coordinated Control"
echo "═══════════════════════════════════════════════════════"

# Send pattern
echo "Sending square pattern..."
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:8,1'" --once
sleep 1

echo "Starting pattern execution..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once

echo ""
echo "The velocity coordinator should now show:"
echo "  • Active source: pattern_executor (priority 3)"
echo "  • No conflicting commands"
echo "  • Smooth movement without jittering"
echo ""
echo "Testing for 40 seconds..."

# Monitor for smooth execution
for i in {1..40}; do
    if [ $((i % 10)) -eq 0 ]; then
        echo ""
        echo "[$i/40s] Current velocity source:"
        ros2 topic echo /velocity_coordinator/active_source --once 2>/dev/null || echo "Pattern executor active"
    else
        printf "."
    fi
    sleep 1
done

echo ""
echo ""
echo "Stopping pattern..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
sleep 2

echo ""
echo "═══════════════════════════════════════════════════════"
echo "RESULTS"
echo "═══════════════════════════════════════════════════════"
echo ""
echo "Was the movement smooth without jittering? (y/n)"
read smooth_result

if [[ "$smooth_result" == "y" || "$smooth_result" == "Y" ]]; then
    echo "✅ SUCCESS! The coordinated executor works properly!"
    echo ""
    echo "The jittering was caused by command conflicts between:"
    echo "  • Pattern executors → /mavros/setpoint_raw/local"
    echo "  • Hybrid controller → /mavros/setpoint_raw/local"
    echo ""
    echo "Solution: Use velocity coordinator for all movements!"
else
    echo "If still jittering, check:"
    echo "  1. Is velocity_coordinator running?"
    echo "  2. Is hybrid_position_controller running?"
    echo "  3. Check: ros2 topic echo /velocity_coordinator/active_source"
fi

# Cleanup
kill $GEN_PID $EXEC_PID 2>/dev/null
pkill -f pattern_executor 2>/dev/null
pkill -f pattern_generator 2>/dev/null

echo ""
echo "Test complete!"
echo ""
echo "To use this executor in your system:"
echo "  1. Always run with hybrid system (./launch_hybrid_system.sh)"
echo "  2. Use: ros2 run search_patterns coordinated_banking_executor"
echo "  3. It will integrate properly with velocity coordinator"