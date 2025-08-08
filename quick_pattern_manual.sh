#!/bin/bash
# Quick manual pattern testing script

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     QUICK MANUAL PATTERN TESTING                          ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Colors
G='\033[0;32m'
Y='\033[1;33m'
B='\033[0;34m'
R='\033[0;31m'
NC='\033[0m'

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
    echo "DRONE:"
    echo "  u) Move up"
    echo "  d) Move down"
    echo "  f) Move forward"
    echo "  b) Move backward"
    echo "  l) Move left"
    echo "  rt) Move right"
    echo "  st) Stop movement"
    echo ""
    echo "OTHER:"
    echo "  m) Monitor status"
    echo "  v) Launch visualizer"
    echo "  q) Quit"
    echo ""
    echo -n "Enter choice: "
}

launch_visualizer() {
    echo "Launching visualizer..."
    gnome-terminal --title="Pattern Visualizer" --geometry=80x30+500+0 -- bash -c "
        source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
        ros2 run search_patterns simple_grid_visualizer 2>/dev/null || \
        ros2 run search_patterns background_visualizer
    " &
    echo "Visualizer launched in new window"
}

monitor_status() {
    echo ""
    echo "Current Pattern Status:"
    timeout 2 ros2 topic echo /pattern_status --once 2>/dev/null || echo "No pattern active"
    echo ""
    echo "Drone Position:"
    timeout 2 ros2 topic echo /mavros/local_position/pose --once 2>/dev/null | grep -A3 "position:" || echo "Position unavailable"
    echo ""
    echo "Velocity Coordinator:"
    timeout 2 ros2 topic echo /velocity_coordinator/active_source --once 2>/dev/null || echo "No active source"
}

# Main loop
while true; do
    show_menu
    read choice
    
    case $choice in
        1)
            echo -e "${G}Sending small square pattern (5m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:5,1'" --once
            ;;
        2)
            echo -e "${G}Sending medium square pattern (10m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:10,2'" --once
            ;;
        3)
            echo -e "${G}Sending large square pattern (15m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:15,3'" --once
            ;;
        4)
            echo -e "${G}Sending small spiral pattern (8m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:8,2'" --once
            ;;
        5)
            echo -e "${G}Sending medium spiral pattern (12m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:12,3'" --once
            ;;
        6)
            echo -e "${G}Sending small zigzag pattern (10x10m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:10,10,3'" --once
            ;;
        7)
            echo -e "${G}Sending medium zigzag pattern (15x15m)...${NC}"
            ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:15,15,5'" --once
            ;;
        s)
            echo -e "${G}Starting pattern execution...${NC}"
            ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
            ;;
        p)
            echo -e "${Y}Pausing pattern...${NC}"
            ros2 topic pub /pattern_control std_msgs/msg/String "data: 'pause'" --once
            ;;
        r)
            echo -e "${G}Resuming pattern...${NC}"
            ros2 topic pub /pattern_control std_msgs/msg/String "data: 'resume'" --once
            ;;
        x)
            echo -e "${R}Stopping pattern...${NC}"
            ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
            ;;
        u)
            echo -e "${B}Moving up...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once
            ;;
        d)
            echo -e "${B}Moving down...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'down'" --once
            ;;
        f)
            echo -e "${B}Moving forward...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once
            ;;
        b)
            echo -e "${B}Moving backward...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'backward'" --once
            ;;
        l)
            echo -e "${B}Moving left...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'left'" --once
            ;;
        rt)
            echo -e "${B}Moving right...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'right'" --once
            ;;
        st)
            echo -e "${R}Stopping movement...${NC}"
            ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once
            ;;
        m)
            monitor_status
            ;;
        v)
            launch_visualizer
            ;;
        q)
            echo -e "${R}Exiting...${NC}"
            exit 0
            ;;
        *)
            echo -e "${R}Invalid choice!${NC}"
            ;;
    esac
done