#!/bin/bash
# Final test with coordinated executor and visualization

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     FINAL PATTERN EXECUTOR TEST                           ║"
echo "║     With Velocity Coordinator + Live Map                  ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Colors
G='\033[0;32m'
Y='\033[1;33m'
B='\033[0;34m'
NC='\033[0m'

echo "This test includes:"
echo "  ✓ Coordinated executor (no conflicts)"
echo "  ✓ Live map visualization"
echo "  ✓ Proper velocity coordinator integration"
echo ""
echo -e "${Y}Prerequisites:${NC}"
echo "  • Hybrid system running (./launch_hybrid_system.sh)"
echo "  • Drone armed and hovering"
echo ""
echo "Press ENTER to start..."
read

# Clean up old pattern nodes
pkill -f pattern_executor 2>/dev/null
pkill -f pattern_generator 2>/dev/null
pkill -f pattern_visualizer 2>/dev/null
pkill -f simple_grid_visualizer 2>/dev/null
sleep 2

# Launch pattern generator
echo -e "${G}[1/3] Starting Pattern Generator...${NC}"
gnome-terminal --title="Pattern Generator" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    ros2 run search_patterns pattern_generator
" &
GEN_PID=$!
sleep 3

# Launch coordinated executor
echo -e "${G}[2/3] Starting Coordinated Banking Executor...${NC}"
gnome-terminal --title="Coordinated Executor" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo '════════════════════════════════════════════════'
    echo '    COORDINATED BANKING EXECUTOR'
    echo '════════════════════════════════════════════════'
    echo ''
    echo 'Topic: /search_pattern/velocity_command'
    echo 'Priority: 4 (search pattern priority)'
    echo 'Status: Ready for patterns'
    echo ''
    ros2 run search_patterns coordinated_banking_executor
" &
EXEC_PID=$!
sleep 3

# Launch visualizer
echo -e "${G}[3/3] Starting Live Map Visualizer...${NC}"
gnome-terminal --title="Live Pattern Map" --geometry=100x30+400+0 -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo '════════════════════════════════════════════════'
    echo '         LIVE PATTERN EXECUTION MAP'
    echo '════════════════════════════════════════════════'
    echo ''
    echo 'Map Legend:'
    echo '  🟢 Green = Planned waypoints'
    echo '  🔵 Blue = Actual drone path'
    echo '  🔴 Red = Current position'
    echo '  🟡 Yellow = Target waypoint'
    echo ''
    echo 'Starting visualizer...'
    echo ''
    # Try simple_grid_visualizer first, fall back to background_visualizer
    ros2 run search_patterns simple_grid_visualizer 2>/dev/null || \
    ros2 run search_patterns background_visualizer 2>/dev/null || \
    ros2 run search_patterns pattern_visualizer
" &
VIS_PID=$!
sleep 5

# Check velocity coordinator
echo ""
echo -e "${B}Checking velocity coordinator status...${NC}"
echo "Current active source:"
ros2 topic echo /velocity_coordinator/active_source --once 2>/dev/null || echo "Waiting for commands..."

echo ""
echo -e "${B}═══════════════════════════════════════════════════════${NC}"
echo -e "${B}TEST 1: Small Square (5m) - Testing basic control${NC}"
echo -e "${B}═══════════════════════════════════════════════════════${NC}"

# Send small square pattern
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:5,1'" --once
sleep 2

echo "Starting pattern execution..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once

echo ""
echo "Watch the map window for drone movement!"
echo "Monitoring for 30 seconds..."
echo ""

# Monitor with progress bar
for i in {1..30}; do
    if [ $((i % 10)) -eq 0 ]; then
        echo ""
        echo "[$i/30s] Velocity source: "
        ros2 topic echo /velocity_coordinator/active_source --once 2>/dev/null | grep "data:" || echo "Pattern executor active"
    else
        printf "▓"
    fi
    sleep 1
done

echo ""
echo ""
echo "Stopping first pattern..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
sleep 3

echo ""
echo -e "${Y}Did the drone move smoothly? (y/n)${NC}"
read response

if [[ "$response" == "y" || "$response" == "Y" ]]; then
    echo -e "${G}✓ Great! Testing larger pattern...${NC}"
    
    echo ""
    echo -e "${B}═══════════════════════════════════════════════════════${NC}"
    echo -e "${B}TEST 2: Spiral Pattern - Testing smooth curves${NC}"
    echo -e "${B}═══════════════════════════════════════════════════════${NC}"
    
    ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:10,3'" --once
    sleep 2
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
    
    echo "Executing spiral for 40 seconds..."
    for i in {1..40}; do
        if [ $((i % 10)) -eq 0 ]; then
            echo "[$i/40s]"
        else
            printf "."
        fi
        sleep 1
    done
    
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
    
    echo ""
    echo -e "${G}═══════════════════════════════════════════════════════${NC}"
    echo -e "${G}SUCCESS! The coordinated executor works!${NC}"
    echo -e "${G}═══════════════════════════════════════════════════════${NC}"
    
else
    echo ""
    echo -e "${Y}Troubleshooting:${NC}"
    echo ""
    echo "1. Check velocity coordinator is receiving commands:"
    echo "   ros2 topic echo /search_pattern/velocity_command"
    echo ""
    echo "2. Check velocity coordinator output:"
    echo "   ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped"
    echo ""
    echo "3. Verify priority system:"
    echo "   ros2 topic echo /velocity_coordinator/active_source"
    echo ""
    echo "4. Try manual control to verify system works:"
    echo "   ros2 topic pub /simple_command std_msgs/msg/String \"data: 'forward'\" --once"
fi

# Final scoring
echo ""
echo -e "${B}═══════════════════════════════════════════════════════${NC}"
echo -e "${B}FINAL EVALUATION${NC}"
echo -e "${B}═══════════════════════════════════════════════════════${NC}"
echo ""
echo "Rate the Coordinated Banking Executor (1-10):"
echo ""
echo "Consider:"
echo "  • No jittering (smooth movement)"
echo "  • Pattern accuracy"
echo "  • Integration with hybrid system"
echo "  • Map visualization working"
echo ""
echo -n "Score (1-10): "
read score

# Save results
RESULT_FILE="final_executor_test_$(date +%Y%m%d_%H%M%S).txt"
echo "Coordinated Banking Executor Test Results" > $RESULT_FILE
echo "Date: $(date)" >> $RESULT_FILE
echo "Score: $score/10" >> $RESULT_FILE
echo "" >> $RESULT_FILE
echo "Comparison:" >> $RESULT_FILE
echo "  precision_pattern_executor: 5/10 (previous best)" >> $RESULT_FILE
echo "  coordinated_banking_executor: $score/10" >> $RESULT_FILE

if [ "$score" -gt 5 ]; then
    echo "" >> $RESULT_FILE
    echo "NEW BEST EXECUTOR!" >> $RESULT_FILE
fi

echo ""
echo "Results saved to: $RESULT_FILE"

# Cleanup
echo ""
echo "Cleaning up..."
kill $GEN_PID $EXEC_PID $VIS_PID 2>/dev/null
pkill -f pattern_executor 2>/dev/null
pkill -f pattern_generator 2>/dev/null
pkill -f visualizer 2>/dev/null

echo ""
echo -e "${G}Test complete!${NC}"
echo ""
echo "Summary:"
echo "  • Executor publishes to: /search_pattern/velocity_command"
echo "  • Velocity coordinator handles: priority management"
echo "  • No conflicts with hybrid system"
echo ""
echo "To use in your demo:"
echo "  1. Run: ./launch_hybrid_system.sh"
echo "  2. Run: ros2 run search_patterns coordinated_banking_executor"
echo "  3. Send patterns as usual"