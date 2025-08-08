#!/bin/bash
# Standalone Visualizer Launcher - Run anytime to see drone movement

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     STANDALONE DRONE VISUALIZER                           ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Colors
G='\033[0;32m'
Y='\033[1;33m'
B='\033[0;34m'
NC='\033[0m'

echo "Select visualizer type:"
echo ""
echo "  1) Simple Grid Visualizer (recommended)"
echo "     - Shows drone position on grid"
echo "     - Tracks path history"
echo "     - Shows waypoints"
echo ""
echo "  2) Background Visualizer"
echo "     - Updates in background"
echo "     - Less CPU intensive"
echo ""
echo "  3) Pattern Visualizer (original)"
echo "     - Full featured"
echo "     - May have focus issues"
echo ""
echo "  4) Terminal Map (no GUI)"
echo "     - ASCII art in terminal"
echo "     - Works over SSH"
echo ""
echo "  5) Web Visualizer"
echo "     - Browser-based at http://localhost:5001"
echo "     - Shows map and controls"
echo ""
echo -n "Enter choice (1-5) [default: 1]: "
read choice

if [ -z "$choice" ]; then
    choice=1
fi

case $choice in
    1)
        echo ""
        echo -e "${G}Launching Simple Grid Visualizer...${NC}"
        echo ""
        echo "Features:"
        echo "  🟢 Green triangle = Start position"
        echo "  🔴 Red dot = Current drone position"
        echo "  🔵 Blue line = Path traveled"
        echo "  🟡 Yellow = Target waypoints"
        echo ""
        ros2 run search_patterns simple_grid_visualizer
        ;;
        
    2)
        echo ""
        echo -e "${G}Launching Background Visualizer...${NC}"
        echo ""
        echo "This visualizer updates every 500ms without stealing focus"
        echo ""
        ros2 run search_patterns background_visualizer
        ;;
        
    3)
        echo ""
        echo -e "${G}Launching Pattern Visualizer...${NC}"
        echo ""
        echo "Using TkAgg backend for better compatibility"
        export MPLBACKEND=TkAgg
        ros2 run search_patterns pattern_visualizer
        ;;
        
    4)
        echo ""
        echo -e "${G}Launching Terminal Map...${NC}"
        echo ""
        echo "ASCII visualization in terminal - no GUI required"
        echo ""
        ros2 run search_patterns terminal_map
        ;;
        
    5)
        echo ""
        echo -e "${G}Launching Web Visualizer...${NC}"
        echo ""
        echo "Starting web server..."
        echo "Open browser to: http://localhost:5001"
        echo ""
        ros2 run search_patterns web_visualizer
        ;;
        
    *)
        echo -e "${Y}Invalid choice. Using Simple Grid Visualizer...${NC}"
        ros2 run search_patterns simple_grid_visualizer
        ;;
esac