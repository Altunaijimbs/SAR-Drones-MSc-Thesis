#!/bin/bash
# Terminal Map - ASCII visualization, no matplotlib needed!

echo "╔══════════════════════════════════════════════════════╗"
echo "║        TERMINAL MAP - ASCII VISUALIZATION            ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This visualizer:"
echo "  • Runs entirely in terminal (no GUI)"
echo "  • No matplotlib required!"
echo "  • Updates every 500ms"
echo "  • Shows ASCII map of drone movement"
echo ""
echo "Map symbols:"
echo "  D = Drone current position"
echo "  S = Start position"
echo "  W = Waypoints"
echo "  . = Path history"
echo "  + = Origin (0,0)"
echo "  · = Grid lines"
echo ""
echo "Prerequisites:"
echo "  - System running (./launch_hybrid_system.sh)"
echo "  - Drone armed and flying"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo "Starting Terminal Map..."
echo "The terminal will clear and show the map"
echo ""
sleep 2

ros2 run search_patterns terminal_map