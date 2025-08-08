#!/bin/bash

# SAR Drone System Launch Script
# This script launches all nodes in the correct order with proper timing

echo "SAR Drone System Launch Script"
echo "=============================="
echo "Make sure Unreal Engine 5.5 with Cosys-AirSim project is OPEN (but don't press Play yet)"
echo ""
read -p "Press Enter when UE5.5 window is open..."

# Function to launch in new terminal
launch_terminal() {
    gnome-terminal --title="$1" -- bash -c "$2; exec bash"
}

# Kill any existing processes
echo "Cleaning up any existing processes..."
pkill -f "px4"
pkill -f "mavros"
pkill -f "ros2"
sleep 2

# Terminal 1: PX4 SITL
echo "Launching PX4 SITL..."
launch_terminal "PX4 SITL" "cd ~/PX4-Autopilot && make px4_sitl none_iris"
echo "Waiting for PX4 to start (5 seconds)..."
sleep 5

echo ""
echo "NOW PRESS PLAY IN UNREAL ENGINE!"
echo "================================"
read -p "Press Enter AFTER pressing Play in UE5..."

# Terminal 2: AirSim ROS2 Wrapper
echo "Launching AirSim ROS2 Wrapper..."
launch_terminal "AirSim ROS2" "cd /home/mbs/Desktop/airsim/Cosys-AirSim/ros2 && source install/setup.bash && ros2 launch airsim_ros_pkgs airsim_node.launch.py"
sleep 5

# Terminal 3: MAVROS (UDP)
echo "Launching MAVROS with UDP connection..."
launch_terminal "MAVROS" "ros2 run mavros mavros_node --ros-args --param fcu_url:='udp://:14550@127.0.0.1:14540' --param target_system_id:=1 --param target_component_id:=1"
sleep 5

# Terminal 4: Smart Keep-Alive
echo "Launching Smart Keep-Alive node..."
launch_terminal "Smart Keep-Alive" "cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws && source install/setup.bash && ros2 run llm_controller smart_keep_alive_node"
sleep 2

# Terminal 5: Velocity Coordinator
echo "Launching Velocity Coordinator..."
launch_terminal "Velocity Coordinator" "cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws && source install/setup.bash && ros2 run search_patterns velocity_coordinator"
sleep 2

# Terminal 6: Arm and Takeoff
echo "Waiting for system to stabilize..."
sleep 5
echo "Setting OFFBOARD mode and arming..."
launch_terminal "Arm & Takeoff" "cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws && source install/setup.bash && \
    ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{custom_mode: 'OFFBOARD'}\" && \
    sleep 2 && \
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\" && \
    sleep 2 && \
    ros2 topic pub /simple_command std_msgs/msg/String \"data: 'up'\" --once && \
    sleep 5 && \
    ros2 topic pub /simple_command std_msgs/msg/String \"data: 'stop'\" --once"

# Terminal 7: Vision System
echo "Launching Vision System..."
launch_terminal "Vision System" "cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws && source install/setup.bash && ros2 launch drone_vision_interpreter vision_pipeline.launch.py camera_topic:=/airsim_node/PX4/front_center_Scene/image"
sleep 2

# Terminal 8: LLM Controller
echo "Launching LLM Controller..."
launch_terminal "LLM Controller" "cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws && source install/setup.bash && ros2 launch llm_controller llm_controller.launch.py velocity_topic:=/llm/velocity_command"
sleep 2

# Terminal 9: Fixed Grid Search
echo "Launching Fixed Grid Search..."
launch_terminal "Fixed Grid Search" "cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws && source install/setup.bash && ros2 run search_patterns fixed_grid_search"
sleep 2

# Terminal 10: Return to Home
echo "Launching Return to Home node..."
launch_terminal "Return to Home" "cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws && source install/setup.bash && ros2 run search_patterns return_to_home"
sleep 2

# Terminal 11: Monitor/Test Commands
echo "Opening terminal for test commands..."
launch_terminal "Test Commands" "cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws && source install/setup.bash && \
    echo 'Test Commands Ready!' && \
    echo '===================' && \
    echo '' && \
    echo 'Start Grid Search:' && \
    echo '  ros2 topic pub /search_command std_msgs/msg/String \"data: search\" --once' && \
    echo '' && \
    echo 'Stop and Return Home:' && \
    echo '  ros2 topic pub /stop_command std_msgs/msg/String \"data: stop\" --once' && \
    echo '' && \
    echo 'Monitor Active Controller:' && \
    echo '  ros2 topic echo /velocity_coordinator/active_source' && \
    echo '' && \
    echo 'Emergency Stop:' && \
    echo '  ros2 topic pub /emergency_stop std_msgs/msg/Bool \"data: true\" --once' && \
    echo ''"

echo ""
echo "All nodes launched! The system should be ready in about 10 seconds."
echo "Check the 'Return to Home' terminal for the HOME POSITION SAVED message."
echo "Use the 'Test Commands' terminal to control the drone."