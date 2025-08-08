#!/bin/bash

# Pattern Executor Comparison System
# Tests all pattern executors and scores their performance

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     PATTERN EXECUTOR COMPARISON & SCORING SYSTEM          ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Results file
RESULTS_FILE="executor_comparison_results_$(date +%Y%m%d_%H%M%S).txt"
echo "Results will be saved to: $RESULTS_FILE"
echo ""

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# List of executors to test
EXECUTORS=(
    "pattern_executor"
    "improved_pattern_executor"
    "attitude_pattern_executor"
    "precision_pattern_executor"
)

# Test patterns (using smaller sizes for safety)
declare -A PATTERNS
PATTERNS["expanding_square"]="10,2"
PATTERNS["spiral"]="12,3"
PATTERNS["zigzag"]="15,15,5"
PATTERNS["lawnmower"]="12,12,4"

# Scoring variables
declare -A EXECUTOR_SCORES
declare -A EXECUTOR_TIMES
declare -A EXECUTOR_SMOOTHNESS

# Function to wait for user confirmation
wait_for_user() {
    echo ""
    echo -e "${YELLOW}Press ENTER when ready to continue...${NC}"
    read
}

# Function to score pattern execution
score_execution() {
    local executor=$1
    local pattern=$2
    
    echo ""
    echo -e "${BLUE}════════════════════════════════════════${NC}"
    echo -e "${YELLOW}SCORING: $executor - $pattern${NC}"
    echo -e "${BLUE}════════════════════════════════════════${NC}"
    echo ""
    echo "Please observe the execution and rate:"
    echo ""
    echo "1. SMOOTHNESS (1-10):"
    echo "   1-3: Jerky, lots of overshooting"
    echo "   4-6: Some oscillation, acceptable"
    echo "   7-9: Smooth with minor issues"
    echo "   10:  Perfect smooth execution"
    echo -n "   Your score: "
    read smoothness_score
    
    echo ""
    echo "2. ACCURACY (1-10):"
    echo "   1-3: Misses waypoints badly"
    echo "   4-6: Gets close to waypoints"
    echo "   7-9: Hits most waypoints well"
    echo "   10:  Perfect waypoint following"
    echo -n "   Your score: "
    read accuracy_score
    
    echo ""
    echo "3. SPEED (1-10):"
    echo "   1-3: Too slow or stops often"
    echo "   4-6: Reasonable speed"
    echo "   7-9: Good consistent speed"
    echo "   10:  Optimal speed throughout"
    echo -n "   Your score: "
    read speed_score
    
    echo ""
    echo "4. TURNING (1-10):"
    echo "   1-3: Bad turns, lots of correction"
    echo "   4-6: Turns work but not smooth"
    echo "   7-9: Good turning behavior"
    echo "   10:  Perfect corner handling"
    echo -n "   Your score: "
    read turning_score
    
    # Calculate total score
    local total_score=$((smoothness_score + accuracy_score + speed_score + turning_score))
    
    echo ""
    echo -e "${GREEN}Total Score: $total_score/40${NC}"
    
    # Save to results
    echo "Executor: $executor | Pattern: $pattern" >> $RESULTS_FILE
    echo "  Smoothness: $smoothness_score/10" >> $RESULTS_FILE
    echo "  Accuracy: $accuracy_score/10" >> $RESULTS_FILE
    echo "  Speed: $speed_score/10" >> $RESULTS_FILE
    echo "  Turning: $turning_score/10" >> $RESULTS_FILE
    echo "  TOTAL: $total_score/40" >> $RESULTS_FILE
    echo "---" >> $RESULTS_FILE
    
    # Update executor total score
    if [ -z "${EXECUTOR_SCORES[$executor]}" ]; then
        EXECUTOR_SCORES[$executor]=$total_score
    else
        EXECUTOR_SCORES[$executor]=$((${EXECUTOR_SCORES[$executor]} + $total_score))
    fi
    
    return $total_score
}

# Function to test one executor
test_executor() {
    local executor=$1
    local executor_index=$2
    local total_executors=$3
    
    echo ""
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║  TESTING EXECUTOR $executor_index/$total_executors: $executor"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo ""
    
    # Kill any existing pattern nodes
    echo "Cleaning up previous nodes..."
    pkill -f pattern_executor
    pkill -f pattern_visualizer
    sleep 2
    
    # Launch the pattern system with the specific executor
    echo -e "${GREEN}Launching $executor with visualization...${NC}"
    
    # Start visualizer in background
    gnome-terminal --title="Pattern Visualizer" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns pattern_visualizer; exec bash" &
    VISUALIZER_PID=$!
    sleep 3
    
    # Start the specific executor in background
    gnome-terminal --title="$executor" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns $executor; exec bash" &
    EXECUTOR_PID=$!
    sleep 3
    
    # Make sure drone is at stable altitude
    echo "Ensuring drone is at proper altitude..."
    ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once
    sleep 2
    
    # Test each pattern
    local pattern_count=0
    for pattern_name in "${!PATTERNS[@]}"; do
        pattern_count=$((pattern_count + 1))
        local pattern_params="${PATTERNS[$pattern_name]}"
        
        echo ""
        echo -e "${BLUE}Pattern $pattern_count/4: $pattern_name ($pattern_params)${NC}"
        echo "-------------------------------------------"
        
        # Send pattern command
        echo "Sending pattern command..."
        ros2 topic pub /pattern_command std_msgs/msg/String "data: '$pattern_name:$pattern_params'" --once
        sleep 1
        
        # Start pattern execution
        echo "Starting pattern execution..."
        ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
        
        # Record start time
        local start_time=$(date +%s)
        
        # Wait for pattern completion or timeout
        echo ""
        echo -e "${YELLOW}Pattern executing...${NC}"
        echo "Watch the visualizer window and observe:"
        echo "  • Smoothness of movement"
        echo "  • Accuracy hitting waypoints"
        echo "  • Speed consistency"
        echo "  • Turn quality at corners"
        echo ""
        echo "Waiting for pattern completion (max 60 seconds)..."
        
        # Monitor pattern status
        local pattern_complete=false
        local timeout=60
        local elapsed=0
        
        while [ $elapsed -lt $timeout ]; do
            # Check if pattern is still running
            status=$(timeout 1 ros2 topic echo /pattern_status --once 2>/dev/null | grep -o "data:.*" | cut -d' ' -f2)
            
            if [[ "$status" == *"complete"* ]] || [[ "$status" == *"idle"* ]]; then
                pattern_complete=true
                break
            fi
            
            sleep 2
            elapsed=$((elapsed + 2))
            echo -n "."
        done
        
        # Record end time
        local end_time=$(date +%s)
        local execution_time=$((end_time - start_time))
        
        echo ""
        if [ "$pattern_complete" = true ]; then
            echo -e "${GREEN}Pattern completed in $execution_time seconds${NC}"
        else
            echo -e "${RED}Pattern timed out after $execution_time seconds${NC}"
        fi
        
        # Score this execution
        score_execution "$executor" "$pattern_name"
        
        # Stop any ongoing pattern
        ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
        sleep 2
        
        # Return drone to center for next pattern
        echo "Returning to center position..."
        ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once
        sleep 3
    done
    
    # Clean up this executor
    echo ""
    echo "Cleaning up $executor..."
    kill $EXECUTOR_PID 2>/dev/null
    kill $VISUALIZER_PID 2>/dev/null
    pkill -f $executor
    pkill -f pattern_visualizer
    sleep 2
}

# Main execution
main() {
    echo "═══════════════════════════════════════════════════════════"
    echo "PREREQUISITES CHECK"
    echo "═══════════════════════════════════════════════════════════"
    echo ""
    echo "Please ensure:"
    echo "  ✓ PX4 SITL is running"
    echo "  ✓ Basic system is launched (./launch_hybrid_system.sh)"
    echo "  ✓ Drone is armed and hovering at ~5m altitude"
    echo ""
    echo -e "${YELLOW}This test will take approximately 30-40 minutes${NC}"
    echo ""
    wait_for_user
    
    # Test initial connection
    echo "Testing ROS2 connection..."
    if ! timeout 2 ros2 topic list > /dev/null 2>&1; then
        echo -e "${RED}ERROR: Cannot connect to ROS2. Is the system running?${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ ROS2 connection OK${NC}"
    
    # Make sure drone is flying
    echo ""
    echo "Checking drone status..."
    echo "If drone is not flying, arm and takeoff now:"
    echo "  ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{custom_mode: 'OFFBOARD'}\""
    echo "  ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\""
    echo "  ros2 topic pub /simple_command std_msgs/msg/String \"data: 'up'\" --once"
    echo ""
    wait_for_user
    
    # Test each executor
    local executor_count=${#EXECUTORS[@]}
    local current_executor=1
    
    for executor in "${EXECUTORS[@]}"; do
        test_executor "$executor" "$current_executor" "$executor_count"
        current_executor=$((current_executor + 1))
        
        if [ $current_executor -le $executor_count ]; then
            echo ""
            echo -e "${YELLOW}Ready for next executor?${NC}"
            wait_for_user
        fi
    done
    
    # Final results
    echo ""
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║                    FINAL RESULTS                          ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo ""
    
    # Calculate and display final scores
    local best_executor=""
    local best_score=0
    
    for executor in "${EXECUTORS[@]}"; do
        local total=${EXECUTOR_SCORES[$executor]}
        local avg=$((total / 4))  # Average across 4 patterns
        
        echo -e "${BLUE}$executor:${NC}"
        echo "  Total Score: $total/160"
        echo "  Average: $avg/40"
        echo ""
        
        if [ $total -gt $best_score ]; then
            best_score=$total
            best_executor=$executor
        fi
    done
    
    echo "═══════════════════════════════════════════════════════════"
    echo -e "${GREEN}🏆 WINNER: $best_executor with $best_score/160 points!${NC}"
    echo "═══════════════════════════════════════════════════════════"
    echo ""
    echo "Detailed results saved to: $RESULTS_FILE"
    
    # Create summary report
    echo "" >> $RESULTS_FILE
    echo "═══════════════════════════════════════════" >> $RESULTS_FILE
    echo "FINAL RANKINGS" >> $RESULTS_FILE
    echo "═══════════════════════════════════════════" >> $RESULTS_FILE
    for executor in "${EXECUTORS[@]}"; do
        local total=${EXECUTOR_SCORES[$executor]}
        echo "$executor: $total/160" >> $RESULTS_FILE
    done
    echo "" >> $RESULTS_FILE
    echo "Winner: $best_executor" >> $RESULTS_FILE
    echo "Test completed: $(date)" >> $RESULTS_FILE
}

# Run main function
main