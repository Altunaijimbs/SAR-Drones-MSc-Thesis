# Search Patterns Package

This package implements various search patterns for SAR (Search and Rescue) drone operations.

## Features

### Grid Search Pattern
- Lawn mower pattern covering rectangular areas
- Configurable spacing and overlap
- Automatic waypoint generation
- Coordinate transformation for AirSim/UE4 compatibility

## Usage

### Build the package
```bash
cd ~/SAR-Drones-MSc-Thesis/ros2_ws
colcon build --packages-select drone_interfaces search_patterns
source install/setup.bash
```

### Launch search patterns
```bash
ros2 launch search_patterns search_patterns.launch.py
```

### Send a grid search command
```bash
# Via natural language (through LLM controller)
"Search a 30x30 meter area with grid pattern"

# Via test script
ros2 run search_patterns test_grid_search
```

## Parameters

- `grid_spacing`: Distance between parallel search lines (default: 10.0m)
- `search_speed`: Drone velocity during search (default: 2.0 m/s)
- `altitude`: Search altitude (default: 20.0m)
- `overlap_percentage`: Overlap between passes (default: 20%)

## Coordinate System

The package handles coordinate transformation between:
- ROS/MAVROS frame (X-forward, Y-right, Z-up)
- AirSim/UE4 frame (X-right, Y-forward, Z-up)

Transformation applied:
- ROS X → UE Y
- ROS Y → UE -X
- ROS Z → UE Z

## Future Patterns
- Spiral search
- Expanding square
- Sector search
- Custom waypoint patterns