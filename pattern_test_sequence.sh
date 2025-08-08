#!/bin/bash
# Automated pattern test sequence

echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║             AUTOMATED PATTERN TEST SEQUENCE                        ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""
echo "This script runs an automated test sequence of all patterns"
echo ""
echo "Prerequisites:"
echo "  - Complete system running (./launch_complete_pattern_system.sh)"
echo "  - Drone armed and hovering at safe altitude"
echo "  - Pattern Map window visible"
echo ""
echo "Press Enter to start automated test sequence..."
read

# Function to run a pattern test
run_pattern_test() {
    local pattern_name=$1
    local pattern_cmd=$2
    local duration=$3
    
    echo ""
    echo "════════════════════════════════════════════════════════"
    echo " Testing: $pattern_name"
    echo "════════════════════════════════════════════════════════"
    echo ""
    
    # Generate pattern
    echo "Generating $pattern_name pattern..."
    ros2 topic pub /pattern_command std_msgs/msg/String "data: '$pattern_cmd'" --once
    sleep 2
    
    # Start execution
    echo "Starting pattern execution..."
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
    
    # Show countdown
    echo -n "Running pattern for $duration seconds: "
    for i in $(seq $duration -1 1); do
        echo -n "$i "
        sleep 1
    done
    echo ""
    
    # Stop pattern
    echo "Stopping pattern..."
    ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
    sleep 3
    
    # Return to center
    echo "Returning to center position..."
    ros2 topic pub /position_command std_msgs/msg/String "data: 'goto:0,0,5'" --once
    sleep 5
}

# Test 1: Small Square
echo ""
echo "TEST 1: SMALL EXPANDING SQUARE"
echo "This creates a small square pattern expanding outward"
run_pattern_test "Small Expanding Square" "expanding_square:6,2" 20

# Test 2: Medium Spiral
echo ""
echo "TEST 2: MEDIUM SPIRAL"
echo "This creates a spiral pattern moving outward from center"
echo "Press Enter to continue..."
read
run_pattern_test "Medium Spiral" "spiral:10,3" 25

# Test 3: Zigzag
echo ""
echo "TEST 3: ZIGZAG PATTERN"
echo "This creates a back-and-forth lawn mower pattern"
echo "Press Enter to continue..."
read
run_pattern_test "Zigzag" "zigzag:12,12,4" 30

# Test 4: Large Square
echo ""
echo "TEST 4: LARGE EXPANDING SQUARE"
echo "This creates a larger square pattern"
echo "Press Enter to continue..."
read
run_pattern_test "Large Expanding Square" "expanding_square:10,3" 35

# Final hover
echo ""
echo "════════════════════════════════════════════════════════"
echo " TEST SEQUENCE COMPLETE"
echo "════════════════════════════════════════════════════════"
echo ""
echo "Returning to hover position..."
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once
echo ""
echo "Pattern test sequence complete!"
echo "Check the Pattern Map to see the paths that were flown."
echo ""
echo "Summary of patterns tested:"
echo "  ✓ Small Expanding Square (6m, 2 loops)"
echo "  ✓ Medium Spiral (10m radius)"
echo "  ✓ Zigzag (12x12m area)"
echo "  ✓ Large Expanding Square (10m, 3 loops)"
echo ""