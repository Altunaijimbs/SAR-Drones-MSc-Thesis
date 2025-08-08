#!/bin/bash

# Pattern Executor Shootout - Quick Comparison with Live Map
# Uses test_pattern_with_map visualization as requested

echo "╔════════════════════════════════════════════════════════════╗"
echo "║         PATTERN EXECUTOR SHOOTOUT 🎯                      ║"
echo "║         Live Map Visualization Included                   ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Quick color codes
G='\033[0;32m'  # Green
Y='\033[1;33m'  # Yellow
B='\033[0;34m'  # Blue
R='\033[0;31m'  # Red
NC='\033[0m'     # No Color

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# The competitors
EXECUTORS=(
    "pattern_executor"
    "improved_pattern_executor"
    "attitude_pattern_executor"
    "precision_pattern_executor"
)

# Test pattern (same for all)
TEST_PATTERN="expanding_square:10,2"  # 10m square, 2 layers

echo -e "${Y}Test Pattern: $TEST_PATTERN${NC}"
echo ""
echo "This will test each executor with the same pattern"
echo "while showing live visualization."
echo ""
echo -e "${Y}Make sure:${NC}"
echo "  • Drone is armed and flying at ~5m"
echo "  • You have enough space for 10m patterns"
echo ""
echo "Press ENTER to start the shootout..."
read

# Results tracking
declare -A SCORES

for i in "${!EXECUTORS[@]}"; do
    executor=${EXECUTORS[$i]}
    echo ""
    echo -e "${B}═══════════════════════════════════════════════════════${NC}"
    echo -e "${B}Round $((i+1))/${#EXECUTORS[@]}: $executor${NC}"
    echo -e "${B}═══════════════════════════════════════════════════════${NC}"
    
    # Clean slate
    pkill -f pattern_executor 2>/dev/null
    pkill -f pattern_generator 2>/dev/null
    pkill -f pattern_visualizer 2>/dev/null
    sleep 2
    
    # Launch the complete pattern system with map
    echo -e "${G}Launching pattern system with live map...${NC}"
    
    # Use test_pattern_with_map approach
    gnome-terminal --title="Pattern Generator" -- bash -c "
        source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
        ros2 run search_patterns pattern_generator
    " &
    GEN_PID=$!
    sleep 2
    
    gnome-terminal --title="$executor" -- bash -c "
        source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
        echo 'Executor: $executor'
        echo 'Waiting for patterns...'
        ros2 run search_patterns $executor
    " &
    EXEC_PID=$!
    sleep 2
    
    # Launch the visualizer map
    gnome-terminal --title="Live Pattern Map" --geometry=100x30+500+0 -- bash -c "
        source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
        echo '═══════════════════════════════════════════'
        echo '     LIVE PATTERN EXECUTION MAP'
        echo '═══════════════════════════════════════════'
        echo ''
        echo 'Legend:'
        echo '  🟢 Green = Planned waypoints'
        echo '  🔵 Blue = Actual path'
        echo '  🔴 Red = Current position'
        echo '  🟡 Yellow = Target waypoint'
        echo ''
        ros2 run search_patterns pattern_visualizer
    " &
    VIS_PID=$!
    sleep 3
    
    # Execute the test pattern
    echo "Sending pattern command..."
    ros2 topic pub /pattern_command std_msgs/msg/String "data: '$TEST_PATTERN'" --once
    sleep 1
    
    echo -e "${Y}Starting pattern execution...${NC}"
    START_TIME=$(date +%s)
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
    
    # Wait and watch
    echo ""
    echo "⏱️  Executing... Watch the map!"
    echo ""
    
    # Monitor for completion (with timeout)
    TIMEOUT=45
    ELAPSED=0
    while [ $ELAPSED -lt $TIMEOUT ]; do
        STATUS=$(timeout 1 ros2 topic echo /pattern_status --once 2>/dev/null | grep -o "data:.*" || echo "running")
        
        if [[ "$STATUS" == *"complete"* ]] || [[ "$STATUS" == *"idle"* ]]; then
            echo -e "${G}✓ Pattern completed!${NC}"
            break
        fi
        
        sleep 1
        ELAPSED=$((ELAPSED + 1))
        printf "\rTime: %ds/%ds" $ELAPSED $TIMEOUT
    done
    
    END_TIME=$(date +%s)
    EXECUTION_TIME=$((END_TIME - START_TIME))
    
    echo ""
    echo ""
    echo -e "${B}Results for $executor:${NC}"
    echo "  Execution time: ${EXECUTION_TIME}s"
    
    # Quick scoring
    echo ""
    echo "Rate this execution (1-10):"
    echo "  Consider: smoothness, accuracy, speed, turning"
    echo -n "  Score: "
    read score
    
    if ! [[ "$score" =~ ^[0-9]+$ ]] || [ "$score" -lt 1 ] || [ "$score" -gt 10 ]; then
        score=5
        echo "  Using default score: 5"
    fi
    
    SCORES[$executor]=$score
    
    # Cleanup
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
    sleep 1
    
    kill $GEN_PID $EXEC_PID $VIS_PID 2>/dev/null
    pkill -f $executor 2>/dev/null
    
    if [ $((i+1)) -lt ${#EXECUTORS[@]} ]; then
        echo ""
        echo -e "${Y}Press ENTER for next executor...${NC}"
        read
    fi
done

# Final results
echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║                   🏆 FINAL RESULTS 🏆                     ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

WINNER=""
BEST_SCORE=0

for executor in "${EXECUTORS[@]}"; do
    score=${SCORES[$executor]}
    echo "  $executor: $score/10"
    
    if [ $score -gt $BEST_SCORE ]; then
        BEST_SCORE=$score
        WINNER=$executor
    fi
done

echo ""
echo "════════════════════════════════════════════════════════════"
echo -e "${G}🎉 WINNER: $WINNER (Score: $BEST_SCORE/10)${NC}"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "Test complete! The $WINNER executor performed best for your use case."