#!/bin/bash
# Fixed Pattern Executor Comparison with Working Visualization

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     PATTERN EXECUTOR COMPARISON (FIXED)                   ║"
echo "║     With Guaranteed Visualization                         ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Colors
G='\033[0;32m'
Y='\033[1;33m'
B='\033[0;34m'
R='\033[0;31m'
NC='\033[0m'

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Executors to test
EXECUTORS=(
    "pattern_executor"
    "improved_pattern_executor"
    "attitude_pattern_executor"
    "precision_pattern_executor"
)

# Test pattern
TEST_PATTERN="expanding_square:8,2"  # 8m square, 2 layers

echo -e "${Y}Test Pattern: $TEST_PATTERN${NC}"
echo ""
echo "This test will:"
echo "  1. Test visualizer first to ensure it's working"
echo "  2. Run each executor with live visualization"
echo "  3. Allow you to score each execution"
echo ""
echo -e "${Y}Prerequisites:${NC}"
echo "  • System is running (./launch_hybrid_system.sh)"
echo "  • Drone is armed and hovering at ~5m"
echo "  • You have space for 8m patterns"
echo ""
echo "Press ENTER to start..."
read

# First, test which visualizer works
echo ""
echo -e "${B}═══════════════════════════════════════════════════════${NC}"
echo -e "${B}STEP 1: Testing Visualizer${NC}"
echo -e "${B}═══════════════════════════════════════════════════════${NC}"

# Kill any existing nodes
pkill -f pattern_executor 2>/dev/null
pkill -f pattern_generator 2>/dev/null
pkill -f visualizer 2>/dev/null
sleep 2

# Launch pattern generator (needed for all tests)
echo "Starting pattern generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    ros2 run search_patterns pattern_generator
" &
GEN_PID=$!
sleep 2

# Launch a test executor
echo "Starting test executor..."
gnome-terminal --title="Test Executor" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    ros2 run search_patterns improved_pattern_executor
" &
TEST_EXEC_PID=$!
sleep 2

# Try simple grid visualizer first (most reliable)
echo "Testing Simple Grid Visualizer..."
gnome-terminal --title="Visualizer Test" --geometry=100x30+400+0 -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo '════════════════════════════════════'
    echo '    SIMPLE GRID VISUALIZER TEST'
    echo '════════════════════════════════════'
    echo ''
    echo 'If you see a plot window with:'
    echo '  • Grid lines'
    echo '  • Drone position markers'
    echo '  • Path tracking'
    echo 'Then the visualizer is working!'
    echo ''
    ros2 run search_patterns simple_grid_visualizer
" &
VIS_TEST_PID=$!
sleep 5

# Send test movement
echo "Sending test movement..."
ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once
sleep 2
ros2 topic pub /simple_command std_msgs/msg/String "data: 'right'" --once
sleep 2
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once

echo ""
echo -e "${Y}Check the visualizer window!${NC}"
echo "Do you see the drone movement on the plot? (y/n)"
read viz_works

VISUALIZER_CMD=""
VISUALIZER_NAME=""

if [[ "$viz_works" == "y" || "$viz_works" == "Y" ]]; then
    echo -e "${G}✓ Simple Grid Visualizer working!${NC}"
    VISUALIZER_CMD="ros2 run search_patterns simple_grid_visualizer"
    VISUALIZER_NAME="Simple Grid Visualizer"
else
    echo "Trying Background Visualizer..."
    kill $VIS_TEST_PID 2>/dev/null
    
    gnome-terminal --title="Visualizer Test 2" --geometry=100x30+400+0 -- bash -c "
        source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
        ros2 run search_patterns background_visualizer
    " &
    VIS_TEST_PID=$!
    sleep 5
    
    ros2 topic pub /simple_command std_msgs/msg/String "data: 'backward'" --once
    sleep 2
    ros2 topic pub /simple_command std_msgs/msg/String "data: 'left'" --once
    sleep 2
    
    echo "Do you see movement in the Background Visualizer? (y/n)"
    read viz2_works
    
    if [[ "$viz2_works" == "y" || "$viz2_works" == "Y" ]]; then
        echo -e "${G}✓ Background Visualizer working!${NC}"
        VISUALIZER_CMD="ros2 run search_patterns background_visualizer"
        VISUALIZER_NAME="Background Visualizer"
    else
        echo -e "${Y}Using Pattern Visualizer with TkAgg backend...${NC}"
        VISUALIZER_CMD="MPLBACKEND=TkAgg ros2 run search_patterns pattern_visualizer"
        VISUALIZER_NAME="Pattern Visualizer"
    fi
fi

# Clean up test nodes
kill $TEST_EXEC_PID $VIS_TEST_PID 2>/dev/null
pkill -f improved_pattern_executor 2>/dev/null
sleep 2

echo ""
echo -e "${B}═══════════════════════════════════════════════════════${NC}"
echo -e "${B}STEP 2: Pattern Executor Comparison${NC}"
echo -e "${B}Using: $VISUALIZER_NAME${NC}"
echo -e "${B}═══════════════════════════════════════════════════════${NC}"
echo ""
echo "Press ENTER to begin executor comparison..."
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
    pkill -f visualizer 2>/dev/null
    sleep 2
    
    # Launch the executor
    echo -e "${G}Launching $executor...${NC}"
    gnome-terminal --title="$executor" -- bash -c "
        source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
        echo 'Executor: $executor'
        echo 'Ready for patterns...'
        ros2 run search_patterns $executor
    " &
    EXEC_PID=$!
    sleep 3
    
    # Launch the working visualizer
    echo -e "${G}Launching $VISUALIZER_NAME...${NC}"
    gnome-terminal --title="Pattern Visualization" --geometry=100x30+400+0 -- bash -c "
        source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
        echo '═══════════════════════════════════════════'
        echo '     PATTERN EXECUTION VISUALIZATION'
        echo '═══════════════════════════════════════════'
        echo ''
        echo 'Executor: $executor'
        echo 'Pattern: $TEST_PATTERN'
        echo ''
        echo 'Watch for:'
        echo '  • Smoothness of movement'
        echo '  • Accuracy at waypoints'
        echo '  • Turn quality'
        echo '  • Overall speed'
        echo ''
        $VISUALIZER_CMD
    " &
    VIS_PID=$!
    sleep 5
    
    # Execute the test pattern
    echo "Sending pattern command..."
    ros2 topic pub /pattern_command std_msgs/msg/String "data: '$TEST_PATTERN'" --once
    sleep 2
    
    echo -e "${Y}Starting pattern execution...${NC}"
    START_TIME=$(date +%s)
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
    
    # Monitor execution
    echo ""
    echo "⏱️  Executing pattern..."
    echo "   Watch the visualization window!"
    echo ""
    
    # Wait for completion with visual feedback
    TIMEOUT=60
    ELAPSED=0
    PATTERN_COMPLETE=false
    
    while [ $ELAPSED -lt $TIMEOUT ]; do
        # Check status
        STATUS=$(timeout 1 ros2 topic echo /pattern_status --once 2>/dev/null | grep -o "data:.*" || echo "")
        
        if [[ "$STATUS" == *"complete"* ]] || [[ "$STATUS" == *"idle"* ]]; then
            PATTERN_COMPLETE=true
            break
        fi
        
        # Visual progress indicator
        printf "\r⏱️  Time: %02d:%02d / %02d:00  " $((ELAPSED/60)) $((ELAPSED%60)) $((TIMEOUT/60))
        
        sleep 1
        ELAPSED=$((ELAPSED + 1))
    done
    
    END_TIME=$(date +%s)
    EXECUTION_TIME=$((END_TIME - START_TIME))
    
    echo ""
    echo ""
    
    if [ "$PATTERN_COMPLETE" = true ]; then
        echo -e "${G}✓ Pattern completed in ${EXECUTION_TIME}s${NC}"
    else
        echo -e "${R}✗ Pattern timeout after ${EXECUTION_TIME}s${NC}"
    fi
    
    # Stop pattern
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
    sleep 2
    
    # Get score
    echo ""
    echo -e "${B}Rate $executor execution (1-10):${NC}"
    echo ""
    echo "Consider:"
    echo "  • Smoothness (no jerky movements)"
    echo "  • Accuracy (hits waypoints)"
    echo "  • Speed (consistent, appropriate)"
    echo "  • Turning (clean corners)"
    echo ""
    echo -n "Score (1-10): "
    read score
    
    if ! [[ "$score" =~ ^[0-9]+$ ]] || [ "$score" -lt 1 ] || [ "$score" -gt 10 ]; then
        score=5
        echo "Using default score: 5"
    fi
    
    SCORES[$executor]=$score
    echo "Execution time: ${EXECUTION_TIME}s, Score: $score/10"
    
    # Cleanup
    kill $EXEC_PID $VIS_PID 2>/dev/null
    pkill -f $executor 2>/dev/null
    
    if [ $((i+1)) -lt ${#EXECUTORS[@]} ]; then
        echo ""
        echo -e "${Y}Ready for next executor? Press ENTER...${NC}"
        read
    fi
done

# Kill pattern generator
kill $GEN_PID 2>/dev/null
pkill -f pattern_generator 2>/dev/null

# Final results
echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║                   🏆 FINAL RESULTS 🏆                     ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

WINNER=""
BEST_SCORE=0

echo "EXECUTOR SCORES:"
echo "----------------"
for executor in "${EXECUTORS[@]}"; do
    score=${SCORES[$executor]}
    printf "%-30s: %2d/10" "$executor" "$score"
    
    if [ $score -gt $BEST_SCORE ]; then
        BEST_SCORE=$score
        WINNER=$executor
        echo " ⭐"
    else
        echo ""
    fi
done

echo ""
echo "════════════════════════════════════════════════════════════"
echo -e "${G}🎉 WINNER: $WINNER (Score: $BEST_SCORE/10)${NC}"
echo "════════════════════════════════════════════════════════════"
echo ""

# Save results
RESULTS_FILE="executor_comparison_$(date +%Y%m%d_%H%M%S).txt"
echo "Pattern Executor Comparison Results" > $RESULTS_FILE
echo "Date: $(date)" >> $RESULTS_FILE
echo "Test Pattern: $TEST_PATTERN" >> $RESULTS_FILE
echo "" >> $RESULTS_FILE
echo "Scores:" >> $RESULTS_FILE
for executor in "${EXECUTORS[@]}"; do
    echo "  $executor: ${SCORES[$executor]}/10" >> $RESULTS_FILE
done
echo "" >> $RESULTS_FILE
echo "Winner: $WINNER (Score: $BEST_SCORE/10)" >> $RESULTS_FILE

echo "Results saved to: $RESULTS_FILE"
echo ""
echo "Comparison complete!"
echo ""
echo "Recommended next steps:"
echo "  1. Use $WINNER for your demo"
echo "  2. Test with larger patterns if needed"
echo "  3. Record demo video with the winning executor"