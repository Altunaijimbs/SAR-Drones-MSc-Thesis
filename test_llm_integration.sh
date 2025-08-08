#!/bin/bash
# Test script for LLM integration

echo "=== LLM Integration Test ==="
echo "This will test natural language drone control"
echo ""

# Check API key
if [ -z "$OPENAI_API_KEY" ]; then
    echo "ERROR: OPENAI_API_KEY not set!"
    echo "Run: export OPENAI_API_KEY='your-key-here'"
    exit 1
fi

echo "✓ OpenAI API key found"
echo ""

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Start LLM controller
echo "Starting Enhanced LLM Controller..."
gnome-terminal --title="LLM Controller" -- bash -c "ros2 run llm_controller enhanced_llm; exec bash"
sleep 3

echo ""
echo "=== Test Commands ==="
echo "Try these natural language commands:"
echo ""
echo "1. Simple search:"
echo "   ros2 topic pub /llm/command_input std_msgs/msg/String \"data: 'Search for people in a 20 meter radius'\" --once"
echo ""
echo "2. Complex mission:"
echo "   ros2 topic pub /llm/command_input std_msgs/msg/String \"data: 'Find 3 missing hikers last seen at position 10, 20, altitude 15 meters'\" --once"
echo ""
echo "3. Hover command:"
echo "   ros2 topic pub /llm/command_input std_msgs/msg/String \"data: 'Hover at current position'\" --once"
echo ""
echo "4. Return home:"
echo "   ros2 topic pub /llm/command_input std_msgs/msg/String \"data: 'Return to base'\" --once"
echo ""
echo "Monitor output in the LLM Controller terminal"
echo ""
echo "To see what the LLM publishes:"
echo "   ros2 topic echo /search_command"
echo "   ros2 topic echo /simple_command"