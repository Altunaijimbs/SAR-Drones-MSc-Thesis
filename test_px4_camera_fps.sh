#!/bin/bash

echo "========================================="
echo "PX4 Camera FPS Test"
echo "========================================="
echo ""

# Test ROS2 image topic frequency
echo "Testing camera topic frequency..."
echo "-----------------------------------------"
timeout 5 ros2 topic hz /airsim_node/PX4/front_center_Scene/image

echo ""
echo "Testing with compressed transport..."
echo "-----------------------------------------"
# Install image transport if needed
if ! ros2 pkg list | grep -q image_transport; then
    echo "Installing image_transport..."
    sudo apt-get install -y ros-humble-image-transport ros-humble-compressed-image-transport
fi

# Test compressed topic
timeout 5 ros2 topic hz /airsim_node/PX4/front_center_Scene/image/compressed

echo ""
echo "Available image topics:"
ros2 topic list | grep -i image

echo ""
echo "========================================="
echo "To use compressed images (higher FPS):"
echo "Subscribe to: /airsim_node/PX4/front_center_Scene/image/compressed"
echo "This typically gives 2-3x better FPS!"
echo "========================================="