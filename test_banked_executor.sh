#!/bin/bash
# Test the new Banked Pure Pursuit Executor

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     BANKED PURE PURSUIT EXECUTOR TEST                     ║"
echo "║     Smooth Banking Turns with No Side-Sliding             ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Colors
G='\033[0;32m'
Y='\033[1;33m'
B='\033[0;34m'
NC='\033[0m'

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo "This executor features:"
echo "  ✓ Pure Pursuit algorithm with dynamic lookahead"
echo "  ✓ Banking turns with proper physics (no side-sliding)"
echo "  ✓ Smooth arc transitions at corners"
echo "  ✓ Speed adaptation (cruise/turn/approach)"
echo "  ✓ Anticipatory turning (starts turns early)"
echo ""
echo -e "${Y}Prerequisites:${NC}"
echo "  • System running (./launch_hybrid_system.sh)"
echo "  • Drone armed and hovering at ~5m"
echo ""
echo "Press ENTER to start test..."
read

# Clean up any existing nodes
pkill -f pattern_executor 2>/dev/null
pkill -f pattern_generator 2>/dev/null
pkill -f visualizer 2>/dev/null
sleep 2

# Launch pattern generator
echo -e "${G}Starting Pattern Generator...${NC}"
gnome-terminal --title="Pattern Generator" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    ros2 run search_patterns pattern_generator
" &
GEN_PID=$!
sleep 3

# Launch the new banked executor
echo -e "${G}Starting Banked Pure Pursuit Executor...${NC}"
gnome-terminal --title="Banked Pure Pursuit Executor" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo '═══════════════════════════════════════════════════════'
    echo '    BANKED PURE PURSUIT EXECUTOR'
    echo '═══════════════════════════════════════════════════════'
    echo ''
    echo 'Features:'
    echo '  • Dynamic lookahead distance'
    echo '  • Banking coordinated turns'
    echo '  • No side-sliding motion'
    echo '  • Smooth arc transitions'
    echo ''
    ros2 run search_patterns banked_pure_pursuit_executor
" &
EXEC_PID=$!
sleep 3

# Launch visualizer
echo -e "${G}Starting Visualizer...${NC}"
gnome-terminal --title="Pattern Visualizer" --geometry=100x30+400+0 -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo 'Pattern Visualizer'
    echo 'Watch for smooth banking turns!'
    echo ''
    ros2 run search_patterns simple_grid_visualizer 2>/dev/null || ros2 run search_patterns background_visualizer
" &
VIS_PID=$!
sleep 3

echo ""
echo -e "${B}═══════════════════════════════════════════════════════${NC}"
echo -e "${B}TEST 1: Square Pattern (Testing 90° turns)${NC}"
echo -e "${B}═══════════════════════════════════════════════════════${NC}"

# Test with square pattern
echo "Sending square pattern command..."
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:10,2'" --once
sleep 2

echo "Starting pattern execution..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once

echo ""
echo "Watch for:"
echo "  • Smooth banking into turns (no stopping)"
echo "  • Drone always facing forward"
echo "  • Arc transitions at corners"
echo "  • Speed reduction before sharp turns"
echo ""
echo "Executing for 40 seconds..."

for i in {1..40}; do
    printf "\rTime: %02d/40s" $i
    sleep 1
done

echo ""
echo ""
echo "Stopping square pattern..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
sleep 3

echo ""
echo -e "${B}═══════════════════════════════════════════════════════${NC}"
echo -e "${B}TEST 2: Spiral Pattern (Testing continuous curves)${NC}"
echo -e "${B}═══════════════════════════════════════════════════════${NC}"

echo "Sending spiral pattern command..."
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:12,3'" --once
sleep 2

echo "Starting pattern execution..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once

echo ""
echo "Watch for:"
echo "  • Smooth continuous banking"
echo "  • Gradual speed changes"
echo "  • No jerky movements"
echo ""
echo "Executing for 30 seconds..."

for i in {1..30}; do
    printf "\rTime: %02d/30s" $i
    sleep 1
done

echo ""
echo ""
echo "Stopping spiral pattern..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
sleep 3

echo ""
echo -e "${B}═══════════════════════════════════════════════════════${NC}"
echo -e "${B}TEST 3: Zigzag Pattern (Testing direction changes)${NC}"
echo -e "${B}═══════════════════════════════════════════════════════${NC}"

echo "Sending zigzag pattern command..."
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:10,10,4'" --once
sleep 2

echo "Starting pattern execution..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once

echo ""
echo "Watch for:"
echo "  • Smooth S-curves at direction changes"
echo "  • No abrupt stops or slides"
echo "  • Anticipatory turning"
echo ""
echo "Executing for 30 seconds..."

for i in {1..30}; do
    printf "\rTime: %02d/30s" $i
    sleep 1
done

echo ""
echo ""
echo "Stopping pattern..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once

echo ""
echo -e "${Y}═══════════════════════════════════════════════════════${NC}"
echo -e "${Y}Performance Evaluation${NC}"
echo -e "${Y}═══════════════════════════════════════════════════════${NC}"
echo ""
echo "Rate the Banked Pure Pursuit Executor (1-10):"
echo ""
echo "Consider:"
echo "  1. Banking turns (no side-sliding)"
echo "  2. Smoothness of transitions"
echo "  3. Speed consistency"
echo "  4. Corner handling"
echo "  5. Overall flight quality"
echo ""
echo -n "Your score (1-10): "
read score

echo ""
echo "Optional feedback (or press ENTER to skip):"
read feedback

# Save results
RESULT_FILE="banked_executor_test_$(date +%Y%m%d_%H%M%S).txt"
echo "Banked Pure Pursuit Executor Test Results" > $RESULT_FILE
echo "Date: $(date)" >> $RESULT_FILE
echo "Score: $score/10" >> $RESULT_FILE
if [ ! -z "$feedback" ]; then
    echo "Feedback: $feedback" >> $RESULT_FILE
fi

echo ""
echo -e "${G}Test complete!${NC}"
echo "Results saved to: $RESULT_FILE"

# Compare with previous best
echo ""
echo "Previous best executor: precision_pattern_executor (5/10)"
echo "Banked Pure Pursuit Executor: $score/10"

if [ "$score" -gt 5 ]; then
    echo -e "${G}🎉 NEW BEST EXECUTOR!${NC}"
    echo "The Banked Pure Pursuit Executor outperforms all previous executors!"
elif [ "$score" -eq 5 ]; then
    echo -e "${Y}Matched the previous best score${NC}"
else
    echo -e "${Y}Room for improvement - may need parameter tuning${NC}"
fi

# Cleanup
echo ""
echo "Cleaning up..."
kill $GEN_PID $EXEC_PID $VIS_PID 2>/dev/null
pkill -f pattern_executor 2>/dev/null
pkill -f pattern_generator 2>/dev/null
pkill -f visualizer 2>/dev/null

echo ""
echo "To use this executor in your demo:"
echo "  ros2 run search_patterns banked_pure_pursuit_executor"
echo ""
echo "To compare with other executors:"
echo "  ./executor_comparison_fixed.sh"