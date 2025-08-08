#!/bin/bash
# SIMPLE TEST FOR COORDINATED EXECUTOR - Everything in one script

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     TEST COORDINATED EXECUTOR - SIMPLE STEPS              ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Colors
G='\033[0;32m'
Y='\033[1;33m'
R='\033[0;31m'
NC='\033[0m'

echo "This script will:"
echo "  1. Start pattern generator"
echo "  2. Start coordinated executor"
echo "  3. Start visualizer"
echo "  4. Send a test pattern"
echo ""

echo -e "${Y}IMPORTANT: Make sure hybrid system is running in another terminal!${NC}"
echo "If not running, open another terminal and run:"
echo "  ./launch_hybrid_system.sh"
echo ""
echo "Press ENTER when hybrid system is running..."
read

# Clean up old nodes
echo "Cleaning up..."
pkill -f pattern_generator 2>/dev/null
pkill -f pattern_executor 2>/dev/null
pkill -f banking_executor 2>/dev/null
sleep 2

# Step 1: Pattern Generator
echo ""
echo -e "${G}STEP 1: Starting Pattern Generator...${NC}"
gnome-terminal --title="Pattern Generator" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo 'PATTERN GENERATOR RUNNING'
    echo '========================'
    ros2 run search_patterns pattern_generator
" &
GEN_PID=$!
sleep 3

# Step 2: Coordinated Executor
echo -e "${G}STEP 2: Starting Coordinated Banking Executor...${NC}"
gnome-terminal --title="Coordinated Executor" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo 'COORDINATED BANKING EXECUTOR RUNNING'
    echo '===================================='
    echo 'Publishing to: /search_pattern/velocity_command'
    echo 'Priority: 4'
    echo ''
    ros2 run search_patterns coordinated_banking_executor
" &
EXEC_PID=$!
sleep 3

# Step 3: Visualizer
echo -e "${G}STEP 3: Starting Visualizer...${NC}"
gnome-terminal --title="Drone Map" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo 'VISUALIZER RUNNING'
    echo '=================='
    ros2 run search_patterns simple_grid_visualizer 2>/dev/null || ros2 run search_patterns background_visualizer
" &
VIS_PID=$!
sleep 3

echo ""
echo -e "${G}═══════════════════════════════════════════════════════${NC}"
echo -e "${G}ALL NODES RUNNING! Now let's test...${NC}"
echo -e "${G}═══════════════════════════════════════════════════════${NC}"
echo ""

# Check if drone is armed
echo "Is the drone armed and flying? (y/n)"
read armed

if [[ "$armed" != "y" && "$armed" != "Y" ]]; then
    echo ""
    echo -e "${Y}Arming drone and taking off...${NC}"
    ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
    sleep 1
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
    sleep 1
    ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once
    echo "Waiting 5 seconds for takeoff..."
    sleep 5
fi

# Test movement first
echo ""
echo -e "${Y}Testing basic movement...${NC}"
ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once
sleep 2
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once
echo "Did the drone move? (y/n)"
read moved

if [[ "$moved" != "y" && "$moved" != "Y" ]]; then
    echo -e "${R}Basic movement not working. Check:${NC}"
    echo "  1. Is hybrid system running?"
    echo "  2. Is drone armed?"
    echo "  3. Check velocity coordinator:"
    ros2 topic echo /velocity_coordinator/active_source --once 2>/dev/null
    echo ""
    echo "Fix these issues first."
else
    echo ""
    echo -e "${G}Great! Now testing patterns...${NC}"
    echo ""
    
    # Send a small test pattern
    echo -e "${Y}Sending small square pattern (5m)...${NC}"
    ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:5,1'" --once
    sleep 2
    
    echo -e "${Y}Starting pattern execution...${NC}"
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
    
    echo ""
    echo -e "${G}PATTERN SHOULD BE EXECUTING NOW!${NC}"
    echo ""
    echo "Watch the visualizer window!"
    echo "The drone should fly in a 5m square pattern."
    echo ""
    echo "Pattern will run for 30 seconds..."
    
    for i in {1..30}; do
        if [ $((i % 5)) -eq 0 ]; then
            echo "[$i/30s] Pattern running..."
            # Check status
            ros2 topic echo /pattern_status --once 2>/dev/null | grep "data:" || true
        fi
        sleep 1
    done
    
    echo ""
    echo -e "${Y}Stopping pattern...${NC}"
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
    
    echo ""
    echo -e "${G}═══════════════════════════════════════════════════════${NC}"
    echo -e "${G}TEST COMPLETE!${NC}"
    echo -e "${G}═══════════════════════════════════════════════════════${NC}"
fi

echo ""
echo "Commands you can use now:"
echo ""
echo "Test other patterns:"
echo "  ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'spiral:10,3'\" --once"
echo "  ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'zigzag:10,10,3'\" --once"
echo ""
echo "Control patterns:"
echo "  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo "  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'stop'\" --once"
echo ""
echo "Check what's happening:"
echo "  ros2 topic echo /pattern_status"
echo "  ros2 topic echo /search_pattern/velocity_command"
echo "  ros2 topic echo /velocity_coordinator/active_source"
echo ""
echo "Press ENTER to clean up and exit..."
read

# Cleanup
echo "Stopping all nodes..."
kill $GEN_PID $EXEC_PID $VIS_PID 2>/dev/null
pkill -f pattern_generator
pkill -f pattern_executor
pkill -f banking_executor

echo "Done!"