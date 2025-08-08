#!/bin/bash
# Launch script with vision system for professor demo

echo "=== SAR DRONE SYSTEM WITH VISION - DEMO VERSION ==="
echo "Make sure:"
echo "1. PX4 is running and waiting for simulator"
echo "2. UE5 is playing"
echo "3. Press Enter to continue..."
read

# Source ROS2
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Terminal 1: AirSim ROS Wrapper
echo "Starting AirSim ROS Wrapper..."
gnome-terminal --title="AirSim ROS" -- bash -c "source /home/mbs/Desktop/airsim/Cosys-AirSim/ros2/install/setup.bash && ros2 launch airsim_ros_pkgs airsim_node.launch.py; exec bash"
sleep 5

# Terminal 2: MAVROS
echo "Starting MAVROS..."
gnome-terminal --title="MAVROS" -- bash -c "ros2 run mavros mavros_node --ros-args --param fcu_url:='udp://:14550@127.0.0.1:14540'; exec bash"
sleep 5

# Terminal 3: Vision Pipeline (YOLO Detection)
echo "Starting Vision System..."
gnome-terminal --title="Vision System" -- bash -c "ros2 launch drone_vision_interpreter vision_pipeline.launch.py camera_topic:=/airsim_node/PX4/front_center_Scene/image; exec bash"
sleep 3

# Terminal 4: Smart Keep-Alive (fixed coordinates)
echo "Starting Smart Keep-Alive..."
gnome-terminal --title="Keep-Alive" -- bash -c "ros2 run llm_controller smart_keep_alive_node; exec bash"
sleep 2

# Terminal 5: Velocity Coordinator (only one!)
echo "Starting Velocity Coordinator..."
gnome-terminal --title="Velocity Coordinator" -- bash -c "ros2 run search_patterns velocity_coordinator; exec bash"
sleep 2

# Terminal 6: RTH Node (with new topic name)
echo "Starting Return to Home..."
gnome-terminal --title="RTH" -- bash -c "ros2 run search_patterns return_to_home; exec bash"
sleep 2

# Terminal 7: Grid Search (fixed coordinates)
echo "Starting Grid Search..."
gnome-terminal --title="Grid Search" -- bash -c "ros2 run search_patterns fixed_grid_search; exec bash"
sleep 2

# Terminal 8: Web Platform (for visualization)
echo "Starting Web Platform..."
gnome-terminal --title="Web Platform" -- bash -c "ros2 run sar_web_platform web_server; exec bash"
sleep 2

echo ""
echo "=== SYSTEM READY WITH VISION ==="
echo ""
echo "FEATURES FOR DEMO:"
echo "  - YOLO object detection running"
echo "  - Web interface at http://localhost:5000"
echo "  - Vision detections published to /vision_detections"
echo ""
echo "COMMANDS:"
echo "  Arm & Takeoff:"
echo "    ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{custom_mode: 'OFFBOARD'}\""
echo "    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\""
echo "    ros2 topic pub /simple_command std_msgs/msg/String \"data: 'up'\" --once"
echo ""
echo "  Movement:"
echo "    ros2 topic pub /simple_command std_msgs/msg/String \"data: 'forward/backward/left/right/up/down/stop'\" --once"
echo ""
echo "  Search Pattern:"
echo "    ros2 topic pub /search_command std_msgs/msg/String \"data: 'search'\" --once"
echo ""
echo "  Return to Home (currently has coordinate issues):"
echo "    ros2 topic pub /rth_command std_msgs/msg/String \"data: 'rth'\" --once"
echo ""
echo "  Emergency Stop:"
echo "    ros2 topic pub /emergency_stop std_msgs/msg/Bool \"data: true\" --once"
echo ""
echo "VISION MONITORING:"
echo "  ros2 topic echo /vision_detections"
echo "  ros2 topic echo /obstacle_detected"