#!/bin/bash

echo "=== RTH MONITOR DASHBOARD ==="
echo "This shows real-time RTH behavior"
echo ""

# Terminal 1: Position
gnome-terminal --title="POSITION" --geometry=60x10+0+0 -- bash -c "
echo 'CURRENT POSITION:';
while true; do
    ros2 topic echo /mavros/local_position/pose --once | grep -A3 'position:' | tail -3;
    sleep 0.5;
done"

# Terminal 2: RTH Debug Messages
gnome-terminal --title="RTH DEBUG" --geometry=80x15+0+300 -- bash -c "
echo 'RTH DEBUG MESSAGES:';
ros2 run search_patterns return_to_home 2>&1 | grep --line-buffered 'RTH DEBUG'"

# Terminal 3: Velocity Commands
gnome-terminal --title="RTH VELOCITIES" --geometry=60x10+600+0 -- bash -c "
echo 'RTH VELOCITY COMMANDS:';
ros2 topic echo /rth/velocity_command"

# Terminal 4: Active Controller
gnome-terminal --title="ACTIVE CONTROLLER" --geometry=60x10+600+300 -- bash -c "
echo 'VELOCITY COORDINATOR STATUS:';
ros2 topic echo /velocity_coordinator/active_source"

echo ""
echo "Monitor windows opened!"
echo "Watch these to understand RTH behavior:"
echo ""
echo "1. POSITION - Shows drone's current X,Y,Z"
echo "2. RTH DEBUG - Shows home position, deltas, and calculated velocities"
echo "3. RTH VELOCITIES - Shows actual velocity commands sent"
echo "4. ACTIVE CONTROLLER - Shows if RTH is in control"
echo ""
echo "WHAT TO LOOK FOR:"
echo "- When RTH activates, delta values show distance to home"
echo "- Velocity X,Y should point TOWARD home (not away)"
echo "- If drone moves backward, velocities have wrong sign"