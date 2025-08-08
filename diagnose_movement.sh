#!/bin/bash
# Diagnose what's actually being sent to the drone

echo "╔════════════════════════════════════════════════════════════╗"
echo "║     MOVEMENT DIAGNOSTICS                                  ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Colors
Y='\033[1;33m'
G='\033[0;32m'
R='\033[0;31m'
NC='\033[0m'

echo "This script will monitor what commands are being sent to the drone"
echo ""

PS3="Select what to monitor: "
options=(
    "Pattern executor output (/search_pattern/velocity_command)"
    "Velocity coordinator output (/mavros/setpoint_velocity/cmd_vel_unstamped)"
    "Raw MAVROS commands (/mavros/setpoint_raw/local)"
    "Current pose (/mavros/local_position/pose)"
    "Compare commanded vs actual velocity"
    "Monitor all simultaneously"
)

select opt in "${options[@]}"
do
    case $opt in
        "Pattern executor output (/search_pattern/velocity_command)")
            echo -e "${Y}Monitoring pattern executor commands...${NC}"
            echo "Look for:"
            echo "  • linear.x and linear.y (should be non-zero for movement)"
            echo "  • angular.z (should be 0 for no rotation)"
            echo ""
            ros2 topic echo /search_pattern/velocity_command
            ;;
            
        "Velocity coordinator output (/mavros/setpoint_velocity/cmd_vel_unstamped)")
            echo -e "${Y}Monitoring velocity coordinator output...${NC}"
            echo "This is what actually goes to the drone"
            echo ""
            ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped
            ;;
            
        "Raw MAVROS commands (/mavros/setpoint_raw/local)")
            echo -e "${Y}Monitoring raw MAVROS commands...${NC}"
            echo ""
            ros2 topic echo /mavros/setpoint_raw/local
            ;;
            
        "Current pose (/mavros/local_position/pose)")
            echo -e "${Y}Monitoring current drone position...${NC}"
            echo "Watch if position.x and position.y change"
            echo ""
            ros2 topic echo /mavros/local_position/pose
            ;;
            
        "Compare commanded vs actual velocity")
            echo -e "${Y}Comparing commanded vs actual velocity...${NC}"
            echo ""
            gnome-terminal --title="Commanded Velocity" -- bash -c "
                echo 'COMMANDED VELOCITY'
                echo '=================='
                ros2 topic echo /search_pattern/velocity_command
            " &
            
            gnome-terminal --title="Actual Velocity" -- bash -c "
                echo 'ACTUAL VELOCITY'
                echo '==============='
                ros2 topic echo /mavros/local_position/velocity_local
            " &
            
            echo "Compare the two windows to see if commands match actual movement"
            ;;
            
        "Monitor all simultaneously")
            echo -e "${G}Opening monitoring terminals...${NC}"
            
            gnome-terminal --geometry=80x15+0+0 --title="Pattern Commands" -- bash -c "
                echo 'PATTERN EXECUTOR COMMANDS'
                echo '========================='
                ros2 topic echo /search_pattern/velocity_command
            " &
            
            gnome-terminal --geometry=80x15+500+0 --title="Coordinator Output" -- bash -c "
                echo 'VELOCITY COORDINATOR OUTPUT'
                echo '==========================='
                ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped
            " &
            
            gnome-terminal --geometry=80x15+0+300 --title="Actual Velocity" -- bash -c "
                echo 'ACTUAL DRONE VELOCITY'
                echo '====================='
                ros2 topic echo /mavros/local_position/velocity_local
            " &
            
            gnome-terminal --geometry=80x15+500+300 --title="Position" -- bash -c "
                echo 'DRONE POSITION'
                echo '=============='
                ros2 topic echo /mavros/local_position/pose --once-per-sec
            " &
            
            echo ""
            echo "Four windows opened to monitor:"
            echo "  1. Pattern executor commands"
            echo "  2. Velocity coordinator output"
            echo "  3. Actual drone velocity"
            echo "  4. Drone position"
            echo ""
            echo "Look for discrepancies between commanded and actual values"
            ;;
            
        *)
            echo "Invalid option"
            ;;
    esac
    break
done