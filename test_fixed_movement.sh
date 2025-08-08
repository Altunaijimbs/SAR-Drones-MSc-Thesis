#!/bin/bash
# Test the FIXED executor - NO YAW rotation, just movement!

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     FIXED MOVEMENT EXECUTOR TEST                          ║"
echo "║     No Yaw Rotation - Pure Position Movement              ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Colors
G='\033[0;32m'
Y='\033[1;33m'
R='\033[0;31m'
NC='\033[0m'

echo -e "${R}IMPORTANT: This executor does NOT control yaw!${NC}"
echo "The drone will move to waypoints without rotating."
echo "This fixes the issue where rotation was being misinterpreted as movement."
echo ""
echo "Press ENTER to start test..."
read

# Clean up
pkill -f pattern_generator 2>/dev/null
pkill -f pattern_executor 2>/dev/null
pkill -f fixed_movement 2>/dev/null
sleep 2

# Start pattern generator
echo -e "${G}Starting Pattern Generator...${NC}"
gnome-terminal --title="Pattern Generator" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    ros2 run search_patterns pattern_generator
" &
GEN_PID=$!
sleep 2

# Start FIXED executor
echo -e "${G}Starting Fixed Movement Executor (NO YAW)...${NC}"
gnome-terminal --title="Fixed Movement Executor" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo '================================'
    echo 'FIXED MOVEMENT EXECUTOR'
    echo 'NO YAW CONTROL - POSITION ONLY'
    echo '================================'
    echo ''
    ros2 run search_patterns fixed_movement_executor
" &
EXEC_PID=$!
sleep 2

# Start visualizer
echo -e "${G}Starting Visualizer...${NC}"
gnome-terminal --title="Position Map" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    ros2 run search_patterns simple_grid_visualizer 2>/dev/null || ros2 run search_patterns background_visualizer
" &
VIS_PID=$!
sleep 2

echo ""
echo -e "${Y}Testing with small square pattern...${NC}"
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:5,1'" --once
sleep 1

echo "Starting pattern..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once

echo ""
echo -e "${G}═══════════════════════════════════════════════════════${NC}"
echo -e "${G}WATCH THE DRONE IN UE5!${NC}"
echo -e "${G}═══════════════════════════════════════════════════════${NC}"
echo ""
echo "You should see:"
echo "  ✓ Drone moving in straight lines"
echo "  ✓ NO continuous rotation/yaw"
echo "  ✓ Clear X,Y movement"
echo "  ✓ Visualizer matching actual movement"
echo ""
echo "Running for 30 seconds..."

for i in {1..30}; do
    if [ $((i % 5)) -eq 0 ]; then
        echo "[$i/30s]"
    fi
    sleep 1
done

echo ""
echo "Stopping pattern..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once

echo ""
echo -e "${Y}Did the drone move properly WITHOUT rotating? (y/n)${NC}"
read result

if [[ "$result" == "y" || "$result" == "Y" ]]; then
    echo ""
    echo -e "${G}SUCCESS! The issue was the yaw control!${NC}"
    echo ""
    echo "The problem was:"
    echo "  • Coordinated executor was trying to rotate AND move"
    echo "  • Small movements + large rotations"
    echo "  • Visualizer showed movement but drone was mostly rotating"
    echo ""
    echo "Solution:"
    echo "  • Fixed executor removes yaw control"
    echo "  • Pure position-based movement"
    echo "  • Clear, predictable patterns"
else
    echo ""
    echo "Debugging: Check velocity commands being sent:"
    echo ""
    timeout 3 ros2 topic echo /search_pattern/velocity_command
fi

# Cleanup
echo ""
echo "Cleaning up..."
kill $GEN_PID $EXEC_PID $VIS_PID 2>/dev/null
pkill -f pattern_generator
pkill -f pattern_executor
pkill -f fixed_movement

echo ""
echo "To use this executor for patterns:"
echo "  ros2 run search_patterns fixed_movement_executor"
echo ""
echo "This executor works best when you don't need the drone to face"
echo "a specific direction, just to follow the pattern path."