#!/bin/bash
# Standalone Pattern Testing - Works without hybrid system

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     STANDALONE PATTERN CONTROL                            ║"
echo "║     No hybrid system required                             ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Colors
G='\033[0;32m'
Y='\033[1;33m'
B='\033[0;34m'
R='\033[0;31m'
NC='\033[0m'

echo -e "${Y}This script starts pattern nodes independently.${NC}"
echo -e "${Y}Make sure MAVROS is running separately.${NC}"
echo ""

# Kill existing pattern nodes
pkill -f pattern_generator 2>/dev/null
pkill -f pattern_executor 2>/dev/null
sleep 1

# Start pattern generator
echo -e "${G}Starting Pattern Generator...${NC}"
ros2 run search_patterns pattern_generator &
GEN_PID=$!
sleep 2

# Select executor
echo ""
echo "Select Pattern Executor:"
echo "  1) Precision Pattern Executor (best standalone)"
echo "  2) Improved Pattern Executor"
echo "  3) Attitude Pattern Executor"
echo "  4) Basic Pattern Executor"
echo ""
echo -n "Enter choice [1]: "
read exec_choice

case $exec_choice in
    1|"")
        EXECUTOR="precision_pattern_executor"
        ;;
    2)
        EXECUTOR="improved_pattern_executor"
        ;;
    3)
        EXECUTOR="attitude_pattern_executor"
        ;;
    4)
        EXECUTOR="pattern_executor"
        ;;
    *)
        EXECUTOR="precision_pattern_executor"
        ;;
esac

echo -e "${G}Starting $EXECUTOR...${NC}"
ros2 run search_patterns $EXECUTOR &
EXEC_PID=$!
sleep 2

# Optional visualizer
echo ""
echo -n "Start visualizer? (y/n) [y]: "
read viz_choice
if [ -z "$viz_choice" ] || [ "$viz_choice" = "y" ]; then
    ros2 run search_patterns simple_grid_visualizer &
    VIS_PID=$!
    echo -e "${G}Visualizer started${NC}"
fi

echo ""
echo -e "${G}═══════════════════════════════════════════════════════${NC}"
echo -e "${G}PATTERN SYSTEM READY${NC}"
echo -e "${G}═══════════════════════════════════════════════════════${NC}"
echo ""

# Quick instructions
cat << EOF
QUICK COMMANDS:

Load Patterns:
  ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:5,1'" --once
  ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:10,3'" --once
  ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:10,10,3'" --once

Control:
  ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
  ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
  ros2 topic pub /pattern_control std_msgs/msg/String "data: 'pause'" --once
  ros2 topic pub /pattern_control std_msgs/msg/String "data: 'resume'" --once

EOF

echo -e "${Y}Press Enter to open interactive menu, or Ctrl+C to use manual commands${NC}"
read

# Interactive menu
while true; do
    echo ""
    echo "═══════════════════════════════════════"
    echo "PATTERN MENU"
    echo "═══════════════════════════════════════"
    echo "1) Square 5m     5) Start pattern"
    echo "2) Square 10m    6) Stop pattern"
    echo "3) Spiral 10m    7) Status"
    echo "4) Zigzag 10m    8) Quit"
    echo ""
    echo -n "Choice: "
    read choice
    
    case $choice in
        1)
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:5,1'" --once
            echo "Square pattern loaded"
            ;;
        2)
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:10,2'" --once
            echo "Square pattern loaded"
            ;;
        3)
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:10,3'" --once
            echo "Spiral pattern loaded"
            ;;
        4)
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:10,10,3'" --once
            echo "Zigzag pattern loaded"
            ;;
        5)
            ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
            echo -e "${G}Pattern started!${NC}"
            ;;
        6)
            ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
            echo -e "${R}Pattern stopped${NC}"
            ;;
        7)
            echo "Pattern status:"
            timeout 1 ros2 topic echo /pattern_status --once 2>/dev/null || echo "No status"
            ;;
        8)
            echo "Stopping nodes..."
            kill $GEN_PID $EXEC_PID $VIS_PID 2>/dev/null
            pkill -f pattern_generator
            pkill -f pattern_executor
            exit 0
            ;;
    esac
done