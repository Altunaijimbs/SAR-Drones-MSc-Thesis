#!/bin/bash
# Debug script to check position data flow

echo "╔══════════════════════════════════════════════════════╗"
echo "║        POSITION DATA FLOW DEBUG                      ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo "1. Checking MAVROS position topic..."
echo "═══════════════════════════════════════"
timeout 2 ros2 topic hz /mavros/local_position/pose

echo ""
echo "2. Checking current position..."
echo "═══════════════════════════════════════"
ros2 topic echo /mavros/local_position/pose --once

echo ""
echo "3. Starting position monitor..."
echo "═══════════════════════════════════════"
echo "Move the drone and watch if numbers change:"
echo ""
ros2 topic echo /mavros/local_position/pose | grep -E "position:|x:|y:|z:"