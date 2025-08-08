# Pattern Map Visualization Guide

You now have **two options** for visualizing drone movement in real-time:

## Option 1: Matplotlib Visualization (Recommended)

### Quick Start:
```bash
# For just the map viewer (works with any drone movement)
./launch_drone_map.sh

# For pattern testing with map
./test_pattern_with_map.sh
```

### Features:
- Real-time drone position tracking (red dot)
- Shows actual flight path (blue line)
- Shows planned waypoints (green markers and dashed line)
- Highlights current target waypoint (yellow dot)
- Auto-scaling view
- Shows altitude in title

### How it works:
- Opens a separate window with live map
- Updates every 100ms
- Stores last 1000 positions
- Works with ALL drone movements (manual, patterns, RTH, etc.)

## Option 2: Web-Based Visualization

### Access:
Open in browser: `http://localhost:5000/static/pattern_map.html`

### Features:
- Same visualization as matplotlib but in browser
- Mouse drag to pan view
- Scroll wheel to zoom
- Click "Toggle Follow" to stop auto-centering
- "Clear Path" button to reset trail

## Usage Examples:

### 1. Monitor Manual Flight:
```bash
# Start map viewer
./launch_drone_map.sh

# Then fly manually with commands like:
ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once
```

### 2. Monitor Pattern Execution:
```bash
# Start pattern system with map
./test_pattern_with_map.sh

# Generate and execute pattern
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:10,2'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
```

### 3. Monitor RTH:
```bash
# Start map viewer
./launch_drone_map.sh

# Trigger RTH and watch the path
ros2 topic pub /rth_command std_msgs/msg/String "data: 'rth'" --once
```

## Map Legend:
- **Blue Line**: Actual drone path (where it has been)
- **Green Dashed Line**: Planned waypoints (where it will go)
- **Green Dots**: Individual waypoints
- **Red Dot**: Current drone position
- **Yellow Dot**: Current target waypoint (during pattern execution)

## Tips:
- The matplotlib version is more reliable and doesn't require WebSocket setup
- Use web version if you want to view from another device
- Map automatically centers on drone unless you pan manually
- Grid lines represent 5-meter squares

## Troubleshooting:
- If map doesn't appear: Check that matplotlib is installed (`pip install matplotlib`)
- If path looks jagged: This is normal - shows actual drone movement including corrections
- If no movement shown: Ensure drone position is being published on `/mavros/local_position/pose`