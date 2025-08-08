#!/bin/bash
# Comprehensive pattern system demonstration

echo "╔════════════════════════════════════════════════════════════╗"
echo "║         SEARCH PATTERN SYSTEM DEMONSTRATION                ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "This demonstrates the new search pattern capabilities"
echo "without modifying any existing working components."
echo ""
echo "═══ PREREQUISITES ═══"
echo "1. Launch full system: ./launch_hybrid_system.sh"
echo "2. Arm and takeoff the drone"
echo "3. Make sure drone is hovering at a safe altitude (>5m)"
echo ""
echo "Press Enter when ready to continue..."
read

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo ""
echo "═══ STARTING PATTERN NODES ═══"
echo ""

# Start Pattern Generator
echo "[1/3] Starting Pattern Generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns pattern_generator; exec bash"
sleep 2

# Start Pattern Executor
echo "[2/3] Starting Pattern Executor..."
gnome-terminal --title="Pattern Executor" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run search_patterns pattern_executor; exec bash"
sleep 2

# Start LLM Bridge (optional)
echo "[3/3] Starting LLM Pattern Bridge (optional)..."
gnome-terminal --title="LLM Pattern Bridge" -- bash -c "source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash && ros2 run llm_controller pattern_llm_bridge; exec bash"
sleep 2

echo ""
echo "═══ PATTERN DEMONSTRATION SEQUENCE ═══"
echo ""
echo "We'll demonstrate three search patterns:"
echo "1. Expanding Square (15m sides, 4 loops)"
echo "2. Spiral (25m radius, 5 turns)"
echo "3. Zigzag (30m x 30m area, 5m spacing)"
echo ""

# Function to run a pattern demo
run_pattern_demo() {
    local pattern_name=$1
    local pattern_cmd=$2
    local wait_time=$3
    
    echo ""
    echo "─── ${pattern_name} Pattern ───"
    echo ""
    echo "Generating pattern..."
    ros2 topic pub /pattern_command std_msgs/msg/String "data: '${pattern_cmd}'" --once
    sleep 2
    
    echo "Starting execution..."
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
    
    echo "Pattern will run for ${wait_time} seconds..."
    echo "(You can monitor progress with: ros2 topic echo /pattern_status)"
    sleep ${wait_time}
    
    echo "Stopping pattern..."
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
    sleep 3
}

echo "Press Enter to start EXPANDING SQUARE pattern demo..."
read
run_pattern_demo "EXPANDING SQUARE" "expanding_square:15,4" 30

echo ""
echo "Press Enter to start SPIRAL pattern demo..."
read
run_pattern_demo "SPIRAL" "spiral:25,5" 30

echo ""
echo "Press Enter to start ZIGZAG pattern demo..."
read
run_pattern_demo "ZIGZAG" "zigzag:30,30,5" 30

echo ""
echo "═══ LLM INTEGRATION TEST ═══"
echo ""
echo "Testing natural language pattern commands via LLM bridge..."
echo ""
echo "Press Enter to test LLM pattern command..."
read

echo "Sending: 'search:expanding_square'"
ros2 topic pub /llm/pattern_request std_msgs/msg/String "data: 'search:expanding_square'" --once
sleep 15

echo "Stopping pattern..."
ros2 topic pub /llm/pattern_request std_msgs/msg/String "data: 'stop_pattern'" --once

echo ""
echo "═══ MANUAL CONTROL COMMANDS ═══"
echo ""
echo "Pattern Generation:"
echo "  ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'expanding_square:SIZE,LOOPS'\" --once"
echo "  ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'spiral:RADIUS,TURNS'\" --once"
echo "  ros2 topic pub /pattern_command std_msgs/msg/String \"data: 'zigzag:WIDTH,HEIGHT,SPACING'\" --once"
echo ""
echo "Pattern Control:"
echo "  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'start'\" --once"
echo "  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'pause'\" --once"
echo "  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'resume'\" --once"
echo "  ros2 topic pub /pattern_control std_msgs/msg/String \"data: 'stop'\" --once"
echo ""
echo "LLM Pattern Commands (via bridge):"
echo "  ros2 topic pub /llm/pattern_request std_msgs/msg/String \"data: 'search:expanding_square'\" --once"
echo "  ros2 topic pub /llm/pattern_request std_msgs/msg/String \"data: 'search:spiral'\" --once"
echo "  ros2 topic pub /llm/pattern_request std_msgs/msg/String \"data: 'stop_pattern'\" --once"
echo ""
echo "Monitoring:"
echo "  ros2 topic echo /pattern_status"
echo "  ros2 topic echo /pattern_waypoints"
echo ""
echo "Demo complete! The pattern system is ready for integration."
echo ""