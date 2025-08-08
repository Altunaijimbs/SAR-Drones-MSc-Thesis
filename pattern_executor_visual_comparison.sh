#!/bin/bash

# Enhanced Pattern Executor Comparison with Guaranteed Visualization
# Tests all executors with live map tracking and automated scoring

echo "╔════════════════════════════════════════════════════════════╗"
echo "║   PATTERN EXECUTOR COMPARISON WITH LIVE VISUALIZATION     ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

# Configuration
RESULTS_DIR="pattern_comparison_$(date +%Y%m%d_%H%M%S)"
mkdir -p $RESULTS_DIR
RESULTS_FILE="$RESULTS_DIR/comparison_results.txt"
METRICS_FILE="$RESULTS_DIR/automated_metrics.csv"

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Executors to test
EXECUTORS=(
    "pattern_executor:Basic waypoint follower"
    "improved_pattern_executor:Enhanced turn handling"
    "attitude_pattern_executor:Face-direction-first"
    "precision_pattern_executor:Variable speed control"
)

# Test patterns - same for all executors for fair comparison
declare -A TEST_PATTERNS
TEST_PATTERNS["square"]="expanding_square:8,2"
TEST_PATTERNS["spiral"]="spiral:10,3"
TEST_PATTERNS["zigzag"]="zigzag:12,12,4"

# Initialize CSV file
echo "Executor,Pattern,Completion_Time,Distance_Error,Path_Smoothness,User_Score,Total_Score" > $METRICS_FILE

# Function to launch visualizer with guaranteed display
launch_visualizer() {
    echo -e "${CYAN}Launching Pattern Visualizer...${NC}"
    
    # Kill any existing visualizer
    pkill -f pattern_visualizer
    pkill -f simple_grid_visualizer
    sleep 1
    
    # Launch the simple grid visualizer (more reliable)
    gnome-terminal --title="Pattern Map Visualizer" --geometry=80x24+0+0 -- bash -c "
        echo '════════════════════════════════════════════════'
        echo '         PATTERN EXECUTION VISUALIZER           '
        echo '════════════════════════════════════════════════'
        echo ''
        echo 'Map Key:'
        echo '  🔴 Red dot = Current drone position'
        echo '  🔵 Blue line = Path traveled'
        echo '  🟢 Green markers = Waypoints'
        echo '  🟡 Yellow = Target waypoint'
        echo ''
        source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
        ros2 run search_patterns simple_grid_visualizer
    " &
    
    VISUALIZER_PID=$!
    sleep 3
    
    # Verify visualizer is running
    if ps -p $VISUALIZER_PID > /dev/null; then
        echo -e "${GREEN}✓ Visualizer launched successfully${NC}"
        return 0
    else
        echo -e "${YELLOW}⚠ Simple grid visualizer failed, trying pattern visualizer...${NC}"
        
        # Try the standard pattern visualizer as fallback
        gnome-terminal --title="Pattern Visualizer" --geometry=80x24+0+0 -- bash -c "
            source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
            ros2 run search_patterns pattern_visualizer
        " &
        VISUALIZER_PID=$!
        sleep 3
    fi
}

# Function to collect automated metrics
collect_metrics() {
    local executor=$1
    local pattern=$2
    local start_time=$3
    local end_time=$4
    
    # Calculate execution time
    local execution_time=$((end_time - start_time))
    
    # Get position data for smoothness calculation (simplified)
    local smoothness_score=0
    local position_data=$(timeout 2 ros2 topic echo /mavros/local_position/pose --once 2>/dev/null)
    if [ ! -z "$position_data" ]; then
        # Simple smoothness heuristic based on current velocity
        local velocity=$(timeout 1 ros2 topic echo /mavros/local_position/velocity_local --once 2>/dev/null | grep -E "x:|y:" | head -2 | awk '{print $2}' | paste -sd, -)
        if [[ "$velocity" == *"0.0"* ]]; then
            smoothness_score=5  # Stopped = less smooth
        else
            smoothness_score=8  # Moving = more smooth
        fi
    fi
    
    echo "$executor,$pattern,$execution_time,0,$smoothness_score" >> $METRICS_FILE
}

# Function to run pattern test
run_pattern_test() {
    local executor_full=$1
    local executor_name=$(echo $executor_full | cut -d: -f1)
    local executor_desc=$(echo $executor_full | cut -d: -f2)
    local pattern_name=$2
    local pattern_cmd=$3
    
    echo ""
    echo -e "${BLUE}Testing: $executor_desc${NC}"
    echo -e "${PURPLE}Pattern: $pattern_name ($pattern_cmd)${NC}"
    echo "-------------------------------------------"
    
    # Clear any previous patterns
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once 2>/dev/null
    sleep 1
    
    # Send pattern command
    echo "Generating pattern..."
    ros2 topic pub /pattern_command std_msgs/msg/String "data: '$pattern_cmd'" --once
    sleep 1
    
    # Start timer
    local start_time=$(date +%s)
    
    # Start pattern execution
    echo "Starting pattern execution..."
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
    
    # Monitor execution with timeout
    echo -e "${YELLOW}⏱ Pattern executing (60s timeout)...${NC}"
    echo "Watch the visualizer window!"
    
    local elapsed=0
    local pattern_complete=false
    while [ $elapsed -lt 60 ]; do
        # Show progress
        printf "\rElapsed: %ds " $elapsed
        
        # Check pattern status
        local status=$(timeout 1 ros2 topic echo /pattern_status --once 2>/dev/null | grep -o "data:.*")
        if [[ "$status" == *"complete"* ]] || [[ "$status" == *"idle"* ]]; then
            pattern_complete=true
            break
        fi
        
        sleep 1
        elapsed=$((elapsed + 1))
    done
    
    # End timer
    local end_time=$(date +%s)
    
    echo ""
    if [ "$pattern_complete" = true ]; then
        echo -e "${GREEN}✓ Pattern completed in $elapsed seconds${NC}"
    else
        echo -e "${RED}✗ Pattern timeout after $elapsed seconds${NC}"
    fi
    
    # Collect automated metrics
    collect_metrics "$executor_name" "$pattern_name" "$start_time" "$end_time"
    
    # Stop pattern
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
    sleep 2
}

# Function to get user scoring
get_user_score() {
    local executor=$1
    
    echo ""
    echo -e "${YELLOW}═══════════════════════════════════════${NC}"
    echo -e "${YELLOW}USER EVALUATION for $executor${NC}"
    echo -e "${YELLOW}═══════════════════════════════════════${NC}"
    echo ""
    echo "Based on what you observed in the visualizer, rate overall performance:"
    echo ""
    echo "  1-2:  Poor (jerky, inaccurate, unusable)"
    echo "  3-4:  Below Average (many issues)"
    echo "  5-6:  Average (acceptable with issues)"
    echo "  7-8:  Good (smooth with minor issues)"
    echo "  9-10: Excellent (smooth, accurate, fast)"
    echo ""
    echo -n "Your score (1-10): "
    read user_score
    
    # Validate input
    if ! [[ "$user_score" =~ ^[0-9]+$ ]] || [ "$user_score" -lt 1 ] || [ "$user_score" -gt 10 ]; then
        echo "Invalid score, using 5"
        user_score=5
    fi
    
    echo "$executor User Score: $user_score/10" >> $RESULTS_FILE
    return $user_score
}

# Main execution
main() {
    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}                    SETUP PHASE                            ${NC}"
    echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
    echo ""
    
    # Check prerequisites
    echo "Checking system status..."
    if ! timeout 2 ros2 topic list > /dev/null 2>&1; then
        echo -e "${RED}ERROR: ROS2 not responding. Please launch the system first:${NC}"
        echo "  ./launch_hybrid_system.sh"
        exit 1
    fi
    
    echo -e "${GREEN}✓ ROS2 system active${NC}"
    echo ""
    echo "Ensure drone is:"
    echo "  • Armed and in OFFBOARD mode"
    echo "  • Hovering at ~5m altitude"
    echo "  • In a clear area for patterns"
    echo ""
    echo -e "${YELLOW}Press ENTER when ready to begin testing...${NC}"
    read
    
    # Launch visualizer first
    launch_visualizer
    
    echo ""
    echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}                 TESTING PHASE                             ${NC}"
    echo -e "${CYAN}═══════════════════════════════════════════════════════════${NC}"
    
    # Test each executor
    local executor_num=1
    declare -A EXECUTOR_TOTAL_SCORES
    
    for executor_full in "${EXECUTORS[@]}"; do
        executor_name=$(echo $executor_full | cut -d: -f1)
        executor_desc=$(echo $executor_full | cut -d: -f2)
        
        echo ""
        echo "╔════════════════════════════════════════════════════════════╗"
        echo "  EXECUTOR $executor_num/${#EXECUTORS[@]}: $executor_desc"
        echo "  ($executor_name)"
        echo "╚════════════════════════════════════════════════════════════╝"
        
        # Kill previous executor if running
        pkill -f "$executor_name" 2>/dev/null
        sleep 1
        
        # Launch this executor
        echo -e "${GREEN}Launching $executor_name...${NC}"
        gnome-terminal --title="$executor_name" --geometry=80x10+0+400 -- bash -c "
            source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
            echo 'Executor: $executor_desc'
            echo 'Starting...'
            ros2 run search_patterns $executor_name
        " &
        EXECUTOR_PID=$!
        sleep 3
        
        # Run each test pattern
        local pattern_total_score=0
        for pattern_key in "${!TEST_PATTERNS[@]}"; do
            run_pattern_test "$executor_full" "$pattern_key" "${TEST_PATTERNS[$pattern_key]}"
            
            # Brief pause between patterns
            sleep 3
        done
        
        # Get user score for this executor
        get_user_score "$executor_desc"
        user_score=$?
        EXECUTOR_TOTAL_SCORES[$executor_name]=$user_score
        
        # Kill this executor
        pkill -f "$executor_name" 2>/dev/null
        
        executor_num=$((executor_num + 1))
        
        if [ $executor_num -le ${#EXECUTORS[@]} ]; then
            echo ""
            echo -e "${YELLOW}Ready for next executor? Press ENTER...${NC}"
            read
        fi
    done
    
    # Kill visualizer
    kill $VISUALIZER_PID 2>/dev/null
    pkill -f pattern_visualizer 2>/dev/null
    pkill -f simple_grid_visualizer 2>/dev/null
    
    # Generate final report
    echo ""
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║                      FINAL RESULTS                        ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo ""
    
    # Find winner
    local best_executor=""
    local best_score=0
    
    for executor_full in "${EXECUTORS[@]}"; do
        executor_name=$(echo $executor_full | cut -d: -f1)
        executor_desc=$(echo $executor_full | cut -d: -f2)
        score=${EXECUTOR_TOTAL_SCORES[$executor_name]}
        
        echo -e "${BLUE}$executor_desc${NC}"
        echo "  Score: $score/10"
        echo ""
        
        if [ $score -gt $best_score ]; then
            best_score=$score
            best_executor=$executor_desc
        fi
    done
    
    echo "═══════════════════════════════════════════════════════════"
    echo -e "${GREEN}🏆 WINNER: $best_executor (Score: $best_score/10)${NC}"
    echo "═══════════════════════════════════════════════════════════"
    echo ""
    echo "Results saved in: $RESULTS_DIR/"
    echo "  • Detailed results: $RESULTS_FILE"
    echo "  • Metrics data: $METRICS_FILE"
    
    # Generate summary
    echo "" >> $RESULTS_FILE
    echo "════════════════════════════════════════" >> $RESULTS_FILE
    echo "SUMMARY" >> $RESULTS_FILE
    echo "Winner: $best_executor (Score: $best_score/10)" >> $RESULTS_FILE
    echo "Test completed: $(date)" >> $RESULTS_FILE
}

# Run the comparison
main