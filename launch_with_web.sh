#!/bin/bash

# SAR Drone System with Web Platform Launch Script

echo "SAR Drone System with Web Control Platform"
echo "=========================================="
echo ""
echo "IMPORTANT: Launch order is critical!"
echo "1. PX4 will start first"
echo "2. You'll be prompted to press Play in UE5"
echo "3. Then all other systems will launch"
echo ""

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Launch the original system
echo "Launching core drone systems..."
./launch_sar_system.sh &

# Wait for systems to stabilize
echo "Waiting for systems to initialize..."
sleep 15

# Launch web platform
echo "Launching web control platform..."
gnome-terminal --title="SAR Web Platform" -- bash -c "cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws && source install/setup.bash && ros2 run sar_web_platform web_server; exec bash"

echo ""
echo "Web platform starting..."
echo "Open your browser and navigate to: http://localhost:5000"
echo ""
echo "Features available:"
echo "- Real-time drone status monitoring"
echo "- Live camera feed with object detection"
echo "- Mission control (search, RTH, emergency stop)"
echo "- Manual flight control with arrow keys"
echo "- Natural language commands"
echo ""
echo "Press Ctrl+C to stop all systems"