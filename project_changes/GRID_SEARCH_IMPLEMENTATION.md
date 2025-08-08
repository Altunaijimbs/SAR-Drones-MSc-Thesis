# Grid Search Pattern Implementation

## Overview
Implemented a grid search pattern for SAR drone operations with proper coordinate transformation between ROS/MAVROS and AirSim/UE4.

## Key Components

### 1. Grid Search Node (`grid_search.py`)
- Generates waypoints for lawn mower pattern
- Handles coordinate transformation (ROS → UE4)
- States: IDLE, MOVING_TO_START, EXECUTING_PATTERN, PAUSED, COMPLETED
- Configurable parameters: spacing, speed, altitude, overlap

### 2. Search Pattern Manager (`search_pattern_manager.py`)
- Parses natural language commands for search patterns
- Bridges LLM controller with search pattern execution
- Extracts parameters from commands (area size, spacing, altitude)

### 3. Message Definitions
- `SearchPattern.msg`: Defines search pattern structure
- Updated `LLMCommand.msg`: Added search_pattern field

### 4. Coordinate Transformation
Based on existing system:
```python
# ROS/MAVROS → AirSim/UE4
transformed_vel.linear.x = -vel_cmd.linear.y  # ROS Y → UE -X
transformed_vel.linear.y = vel_cmd.linear.x   # ROS X → UE Y
transformed_vel.linear.z = vel_cmd.linear.z   # ROS Z → UE Z
```

## Usage Examples

### Natural Language Commands
- "Search a 50x50 meter area with grid pattern"
- "Perform grid search with 5 meter spacing at 30 meter altitude"
- "Search the area ahead"

### Direct Testing
```bash
# Launch search patterns
ros2 launch search_patterns search_patterns.launch.py

# Run test command
ros2 run search_patterns test_grid_search
```

## Integration Points

1. **With LLM Controller**: Listens to `/drone/llm_response` for search commands
2. **With MAVROS**: Publishes to `/mavros/setpoint_velocity/cmd_vel_unstamped`
3. **With Vision System**: Can pause/resume based on detections

## Next Steps
- Implement spiral search pattern
- Add expanding square pattern
- Integrate with obstacle avoidance
- Add search progress visualization