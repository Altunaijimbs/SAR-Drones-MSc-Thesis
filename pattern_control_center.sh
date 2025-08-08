#!/bin/bash
# Complete Pattern Control Center - Actually starts the necessary nodes!

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     PATTERN CONTROL CENTER                                ║"
echo "║     Complete system with all nodes running                ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Colors
G='\033[0;32m'
Y='\033[1;33m'
B='\033[0;34m'
R='\033[0;31m'
NC='\033[0m'

# Check if hybrid system is running
echo "Checking system status..."
echo ""

# Try to check ROS2 with timeout
if timeout 2 ros2 topic list > /dev/null 2>&1; then
    echo -e "${G}✓ ROS2 system detected${NC}"
    SYSTEM_RUNNING=true
else
    echo -e "${Y}⚠ ROS2 system may not be fully running${NC}"
    echo ""
    echo "Options:"
    echo "  1) The hybrid system is running (continue anyway)"
    echo "  2) I need to start the hybrid system first (exit)"
    echo ""
    echo -n "Enter choice [1]: "
    read sys_choice
    
    if [ "$sys_choice" = "2" ]; then
        echo ""
        echo "Please run in another terminal:"
        echo "  ./launch_hybrid_system.sh"
        echo ""
        echo "Then run this script again."
        exit 0
    fi
    
    echo ""
    echo -e "${Y}Continuing...${NC}"
    SYSTEM_RUNNING=false
fi

echo ""

# Kill any existing pattern nodes
echo "Cleaning up old pattern nodes..."
pkill -f pattern_generator 2>/dev/null
pkill -f pattern_executor 2>/dev/null
sleep 2

# Start pattern generator
echo -e "${Y}Starting Pattern Generator...${NC}"
gnome-terminal --title="Pattern Generator" --geometry=80x10+0+0 -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo 'Pattern Generator Running'
    echo '========================'
    ros2 run search_patterns pattern_generator
" &
GEN_PID=$!
sleep 3

# Ask which executor to use
echo ""
echo "Select Pattern Executor:"
echo "  1) Coordinated Banking Executor (works with hybrid system)"
echo "  2) Precision Pattern Executor (your best standalone - 5/10)"
echo "  3) Improved Pattern Executor (basic improved - 4/10)"
echo ""
echo -n "Enter choice [1]: "
read executor_choice

if [ -z "$executor_choice" ]; then
    executor_choice=1
fi

case $executor_choice in
    1)
        EXECUTOR="coordinated_banking_executor"
        EXECUTOR_NAME="Coordinated Banking Executor"
        ;;
    2)
        EXECUTOR="precision_pattern_executor"
        EXECUTOR_NAME="Precision Pattern Executor"
        ;;
    3)
        EXECUTOR="improved_pattern_executor"
        EXECUTOR_NAME="Improved Pattern Executor"
        ;;
    *)
        EXECUTOR="coordinated_banking_executor"
        EXECUTOR_NAME="Coordinated Banking Executor"
        ;;
esac

# Start selected executor
echo -e "${Y}Starting $EXECUTOR_NAME...${NC}"
gnome-terminal --title="$EXECUTOR_NAME" --geometry=80x10+0+150 -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo '$EXECUTOR_NAME Running'
    echo '========================'
    ros2 run search_patterns $EXECUTOR
" &
EXEC_PID=$!
sleep 3

# Optional: Start visualizer
echo ""
echo -n "Launch visualizer? (y/n) [y]: "
read viz_choice
if [ -z "$viz_choice" ] || [ "$viz_choice" = "y" ] || [ "$viz_choice" = "Y" ]; then
    echo -e "${Y}Starting Visualizer...${NC}"
    gnome-terminal --title="Drone Position Map" --geometry=80x30+500+0 -- bash -c "
        source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
        ros2 run search_patterns simple_grid_visualizer 2>/dev/null || \
        ros2 run search_patterns background_visualizer
    " &
    VIS_PID=$!
fi

echo ""
echo -e "${G}═══════════════════════════════════════════════════════${NC}"
echo -e "${G}SYSTEM READY!${NC}"
echo -e "${G}═══════════════════════════════════════════════════════${NC}"
echo ""
echo "Nodes running:"
echo "  ✓ Pattern Generator"
echo "  ✓ $EXECUTOR_NAME"
if [ ! -z "$VIS_PID" ]; then
    echo "  ✓ Visualizer"
fi
echo ""

# Function to show menu
show_menu() {
    echo ""
    echo -e "${B}═══════════════════════════════════════════════════════${NC}"
    echo -e "${B}PATTERN CONTROL MENU${NC}"
    echo -e "${B}═══════════════════════════════════════════════════════${NC}"
    echo ""
    echo "PATTERNS:"
    echo "  1) Small Square (5m)"
    echo "  2) Medium Square (10m)"
    echo "  3) Large Square (15m)"
    echo "  4) Small Spiral (8m)"
    echo "  5) Medium Spiral (12m)"
    echo "  6) Small Zigzag (10x10m)"
    echo "  7) Medium Zigzag (15x15m)"
    echo ""
    echo "CONTROL:"
    echo "  s) Start pattern"
    echo "  p) Pause pattern"
    echo "  r) Resume pattern"
    echo "  x) Stop pattern"
    echo ""
    echo "DRONE MOVEMENT:"
    echo "  up) Move up"
    echo "  dn) Move down"
    echo "  fw) Move forward"
    echo "  bk) Move backward"
    echo "  lt) Move left"
    echo "  rt) Move right"
    echo "  st) Stop movement"
    echo ""
    echo "STATUS:"
    echo "  m) Monitor status"
    echo "  t) Test drone movement"
    echo ""
    echo "  q) Quit (stops all nodes)"
    echo ""
    echo -n "Enter choice: "
}

test_movement() {
    echo ""
    echo -e "${Y}Testing drone movement...${NC}"
    echo "Sending forward command..."
    ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once
    sleep 2
    echo "Sending stop command..."
    ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once
    echo -e "${G}If drone moved, system is working!${NC}"
}

monitor_status() {
    echo ""
    echo -e "${Y}System Status:${NC}"
    echo "─────────────────────────────"
    
    echo "Pattern Status:"
    timeout 1 ros2 topic echo /pattern_status --once 2>/dev/null || echo "  No pattern active"
    
    echo ""
    echo "Velocity Coordinator:"
    timeout 1 ros2 topic echo /velocity_coordinator/active_source --once 2>/dev/null || echo "  No active source"
    
    echo ""
    echo "Current Position:"
    timeout 1 ros2 topic echo /mavros/local_position/pose --once 2>/dev/null | grep -A3 "position:" || echo "  Position unavailable"
}

# Main control loop
while true; do
    show_menu
    read choice
    
    case $choice in
        1)
            echo -e "${G}Sending small square pattern (5m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:5,1'" --once
            echo "Pattern loaded. Press 's' to start execution."
            ;;
        2)
            echo -e "${G}Sending medium square pattern (10m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:10,2'" --once
            echo "Pattern loaded. Press 's' to start execution."
            ;;
        3)
            echo -e "${G}Sending large square pattern (15m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:15,3'" --once
            echo "Pattern loaded. Press 's' to start execution."
            ;;
        4)
            echo -e "${G}Sending small spiral pattern (8m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:8,2'" --once
            echo "Pattern loaded. Press 's' to start execution."
            ;;
        5)
            echo -e "${G}Sending medium spiral pattern (12m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:12,3'" --once
            echo "Pattern loaded. Press 's' to start execution."
            ;;
        6)
            echo -e "${G}Sending small zigzag pattern (10x10m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:10,10,3'" --once
            echo "Pattern loaded. Press 's' to start execution."
            ;;
        7)
            echo -e "${G}Sending medium zigzag pattern (15x15m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:15,15,5'" --once
            echo "Pattern loaded. Press 's' to start execution."
            ;;
        s)
            echo -e "${G}▶ Starting pattern execution...${NC}"
            ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
            echo "Pattern started! Watch the visualizer window."
            ;;
        p)
            echo -e "${Y}⏸ Pausing pattern...${NC}"
            ros2 topic pub /pattern_control std_msgs/msg/String "data: 'pause'" --once
            ;;
        r)
            echo -e "${G}▶ Resuming pattern...${NC}"
            ros2 topic pub /pattern_control std_msgs/msg/String "data: 'resume'" --once
            ;;
        x)
            echo -e "${R}⏹ Stopping pattern...${NC}"
            ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
            ;;
        up)
            echo -e "${B}↑ Moving up...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once
            ;;
        dn)
            echo -e "${B}↓ Moving down...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'down'" --once
            ;;
        fw)
            echo -e "${B}→ Moving forward...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once
            ;;
        bk)
            echo -e "${B}← Moving backward...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'backward'" --once
            ;;
        lt)
            echo -e "${B}← Moving left...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'left'" --once
            ;;
        rt)
            echo -e "${B}→ Moving right...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'right'" --once
            ;;
        st)
            echo -e "${R}⏹ Stopping movement...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once
            ;;
        m)
            monitor_status
            ;;
        t)
            test_movement
            ;;
        q)
            echo -e "${R}Shutting down...${NC}"
            kill $GEN_PID $EXEC_PID 2>/dev/null
            [ ! -z "$VIS_PID" ] && kill $VIS_PID 2>/dev/null
            pkill -f pattern_generator 2>/dev/null
            pkill -f pattern_executor 2>/dev/null
            echo "All pattern nodes stopped."
            exit 0
            ;;
        *)
            echo -e "${R}Invalid choice!${NC}"
            ;;
    esac
done