#!/bin/bash
# Test script to fix and verify visualizer is showing drone movement

echo "╔══════════════════════════════════════════════════════╗"
echo "║        VISUALIZER FIX AND TEST                       ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

echo "This script will:"
echo "  1. Launch the pattern generator"
echo "  2. Launch a pattern executor"
echo "  3. Launch the visualizer with proper setup"
echo "  4. Send a test pattern"
echo ""
echo "Prerequisites:"
echo "  • System is running (./launch_hybrid_system.sh)"
echo "  • Drone is armed and hovering at ~5m"
echo ""
echo "Press ENTER to continue..."
read

# Kill any existing nodes
echo "Cleaning up existing nodes..."
pkill -f pattern_executor
pkill -f pattern_generator
pkill -f pattern_visualizer
pkill -f simple_grid_visualizer
pkill -f background_visualizer
sleep 2

# Launch pattern generator
echo ""
echo "1. Launching Pattern Generator..."
gnome-terminal --title="Pattern Generator" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo 'Pattern Generator Starting...'
    ros2 run search_patterns pattern_generator
" &
GEN_PID=$!
sleep 3

# Launch improved pattern executor (most reliable)
echo "2. Launching Improved Pattern Executor..."
gnome-terminal --title="Pattern Executor" -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo 'Improved Pattern Executor Starting...'
    ros2 run search_patterns improved_pattern_executor
" &
EXEC_PID=$!
sleep 3

# Launch the simple grid visualizer (most reliable visualizer)
echo "3. Launching Simple Grid Visualizer..."
gnome-terminal --title="Drone Position Visualizer" --geometry=80x30+500+0 -- bash -c "
    source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
    echo '═══════════════════════════════════════════'
    echo '     DRONE POSITION VISUALIZER'
    echo '═══════════════════════════════════════════'
    echo ''
    echo 'This visualizer will show:'
    echo '  🟢 Green triangle = Start position'
    echo '  🔴 Red dot = Current drone position'
    echo '  🔵 Blue line = Path traveled'
    echo '  🟡 Yellow markers = Target waypoints'
    echo ''
    echo 'Starting visualizer...'
    ros2 run search_patterns simple_grid_visualizer
" &
VIS_PID=$!
sleep 5

echo ""
echo "═══════════════════════════════════════════════════════"
echo "Testing Visualizer with Small Square Pattern"
echo "═══════════════════════════════════════════════════════"
echo ""

# Send a small test pattern
echo "4. Sending test pattern (5m square)..."
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:5,1'" --once
sleep 2

echo "5. Starting pattern execution..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║  WATCH THE VISUALIZER WINDOW NOW!                         ║"
echo "║                                                            ║"
echo "║  You should see:                                          ║"
echo "║  • A plot window showing the drone position               ║"
echo "║  • The drone moving in a square pattern                   ║"
echo "║  • Blue line tracing the path                            ║"
echo "║  • Red dot showing current position                      ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Monitor for 30 seconds
echo "Monitoring for 30 seconds..."
for i in {1..30}; do
    printf "\rTime: %d/30s" $i
    sleep 1
done

echo ""
echo ""
echo "Stopping pattern..."
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once

echo ""
echo "Did you see the drone movement in the visualizer? (y/n)"
read response

if [[ "$response" == "y" || "$response" == "Y" ]]; then
    echo ""
    echo "✅ Great! The visualizer is working correctly."
    echo ""
    echo "You can now run the pattern executor comparison:"
    echo "  ./executor_shootout.sh"
    echo ""
else
    echo ""
    echo "❌ Visualizer not showing movement. Let's try the background visualizer instead..."
    
    # Kill simple grid visualizer
    kill $VIS_PID 2>/dev/null
    pkill -f simple_grid_visualizer
    sleep 2
    
    # Try background visualizer
    echo "Launching Background Visualizer..."
    gnome-terminal --title="Background Visualizer" --geometry=80x30+500+0 -- bash -c "
        source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
        echo 'Starting Background Visualizer...'
        ros2 run search_patterns background_visualizer
    " &
    VIS2_PID=$!
    sleep 3
    
    echo "Testing with another small pattern..."
    ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:8,2'" --once
    sleep 1
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
    
    echo ""
    echo "Check the new visualizer window for 20 seconds..."
    sleep 20
    
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
    
    echo ""
    echo "Did the background visualizer work? (y/n)"
    read response2
    
    if [[ "$response2" == "y" || "$response2" == "Y" ]]; then
        echo "✅ Use background_visualizer in your pattern tests"
    else
        echo "❌ Visualization issue persists. Checking pattern_visualizer..."
        
        # Kill background visualizer
        kill $VIS2_PID 2>/dev/null
        pkill -f background_visualizer
        sleep 2
        
        # Try original pattern visualizer
        echo "Launching Original Pattern Visualizer..."
        python3 -c "import matplotlib; matplotlib.use('TkAgg')" 2>/dev/null
        
        gnome-terminal --title="Pattern Visualizer" --geometry=80x30+500+0 -- bash -c "
            source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
            echo 'Starting Pattern Visualizer...'
            export MPLBACKEND=TkAgg
            ros2 run search_patterns pattern_visualizer
        " &
        VIS3_PID=$!
        sleep 3
        
        echo "Final test with zigzag pattern..."
        ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:10,10,3'" --once
        sleep 1
        ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
        
        echo "Check this visualizer for 20 seconds..."
        sleep 20
        
        ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
    fi
fi

echo ""
echo "Cleaning up..."
kill $GEN_PID $EXEC_PID 2>/dev/null
pkill -f pattern_executor
pkill -f pattern_generator
pkill -f visualizer

echo ""
echo "Test complete!"
echo ""
echo "If visualization is still not working, try:"
echo "  1. Check if matplotlib is installed: python3 -c 'import matplotlib'"
echo "  2. Set backend: export MPLBACKEND=TkAgg"
echo "  3. Use terminal monitor instead: ./launch_pattern_monitor.sh"