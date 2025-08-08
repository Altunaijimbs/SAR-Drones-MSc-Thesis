#!/bin/bash
# Test the web-based visualizer - runs in browser, zero interference

echo "╔══════════════════════════════════════════════════════╗"
echo "║        WEB-BASED VISUALIZER TEST                     ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This visualizer:"
echo "  • Runs in your web browser"
echo "  • ZERO terminal/window interference"
echo "  • Auto-refreshes every 2 seconds"
echo "  • Interactive plotly graphs"
echo ""
echo "Prerequisites:"
echo "  - System running (./launch_hybrid_system.sh)"
echo "  - Drone can be on ground or already flying"
echo "  - Python packages: flask, plotly (pip install flask plotly)"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo "Starting Web Visualizer..."
echo ""
echo "════════════════════════════════════════════════════════"
echo "    Open your browser and go to: http://localhost:8050"
echo "════════════════════════════════════════════════════════"
echo ""
echo "The page will auto-refresh every 2 seconds showing:"
echo "  • Current drone position"
echo "  • Path history (blue line)"
echo "  • Start position (green triangle)"
echo "  • Pattern waypoints (if active)"
echo ""
echo "Move the drone with these commands in another terminal:"
echo "  ros2 topic pub /simple_command std_msgs/msg/String \"data: 'forward'\" --once"
echo "  ros2 topic pub /simple_command std_msgs/msg/String \"data: 'right'\" --once"
echo ""
echo "Press Ctrl+C to stop the visualizer"
echo ""

ros2 run search_patterns web_visualizer