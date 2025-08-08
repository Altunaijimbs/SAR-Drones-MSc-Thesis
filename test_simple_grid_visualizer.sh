#!/bin/bash
# Test the simple grid visualizer that works with already-flying drone

echo "╔══════════════════════════════════════════════════════╗"
echo "║        SIMPLE GRID VISUALIZER TEST                   ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This visualizer:"
echo "  • Works even if drone is already flying"
echo "  • Shows current altitude correctly"
echo "  • Marks where tracking started with green triangle"
echo "  • Updates every 100ms with grid display"
echo ""
echo "Prerequisites:"
echo "  - System running (./launch_hybrid_system.sh)"
echo "  - Drone can be on ground or already flying"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo "Starting Simple Grid Visualizer..."
echo ""
echo "The plot will show:"
echo "  🟢 Green triangle = Start position when visualizer launched"
echo "  🔴 Red dot = Current drone position"
echo "  🔵 Blue line = Path traveled since launch"
echo "  📊 Grid = Reference grid with 0,0 marked"
echo "  📝 Info box = Current position and delta from start"
echo ""
echo "Test by moving the drone:"
echo "  ros2 topic pub /simple_command std_msgs/msg/String \"data: 'forward'\" --once"
echo "  ros2 topic pub /simple_command std_msgs/msg/String \"data: 'right'\" --once"
echo ""

ros2 run search_patterns simple_grid_visualizer