#!/bin/bash
# Minimal launch for attitude-based pattern control
# Only essential nodes - no competing controllers!

echo "╔══════════════════════════════════════════════════════╗"
echo "║    MINIMAL SYSTEM FOR ATTITUDE PATTERNS              ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "This launches ONLY:"
echo "  ✓ AirSim connection"
echo "  ✓ MAVROS"
echo "  ✓ Web interface (optional)"
echo ""
echo "NO velocity coordinator or position controllers!"
echo "The attitude executor has FULL control"
echo ""
echo "Prerequisites:"
echo "  - PX4 running"
echo "  - UE5 playing"
echo ""
echo "Press Enter to start minimal system..."
read

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Terminal 1: AirSim ROS Wrapper
echo "[1/3] Starting AirSim ROS Wrapper..."
gnome-terminal --title="AirSim ROS" -- bash -c "source /home/mbs/Desktop/airsim/Cosys-AirSim/ros2/install/setup.bash && ros2 launch airsim_ros_pkgs airsim_node.launch.py; exec bash"
echo "      Waiting 7 seconds for AirSim initialization..."
sleep 7

# Terminal 2: MAVROS
echo "[2/3] Starting MAVROS..."
gnome-terminal --title="MAVROS" -- bash -c "ros2 run mavros mavros_node --ros-args --param fcu_url:='udp://:14550@127.0.0.1:14540'; exec bash"
echo "      Waiting 5 seconds for MAVROS connection..."
sleep 5

# Terminal 3: Web Platform (optional, for video feed)
echo "[3/3] Starting Web Platform (optional)..."
gnome-terminal --title="Web Platform" -- bash -c "ros2 run sar_web_platform web_server; exec bash"
sleep 2

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║        MINIMAL SYSTEM READY                          ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "Now you can run attitude patterns WITHOUT conflicts!"
echo ""
echo "1. ARM & TAKEOFF:"
echo "   ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{custom_mode: 'OFFBOARD'}\""
echo "   ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\""
echo ""
echo "2. MANUAL TAKEOFF (since no keep-alive):"
echo "   Use QGroundControl or:"
echo "   ros2 topic pub /mavros/setpoint_raw/local mavros_msgs/msg/PositionTarget \\"
echo "     \"position: {x: 0.0, y: 0.0, z: 5.0}\" --rate 10"
echo ""
echo "3. Then run: ./test_attitude_patterns.sh"
echo ""
echo "The attitude executor will have FULL control - no conflicts!"