# Search Pattern Integration Guide

## Overview
This guide explains how to integrate the new search pattern system with your existing LLM controller without breaking current functionality.

## Current Architecture

### Standalone Pattern System (New)
- **Pattern Generator** (`pattern_generator.py`): Creates waypoint sequences for various patterns
- **Pattern Executor** (`pattern_executor.py`): Follows waypoints using position commands
- **Pattern LLM Bridge** (`pattern_llm_bridge.py`): Optional bridge for LLM integration

### Existing System (Working)
- **Practical LLM Controller**: Handles natural language commands
- **Hybrid Position Controller**: Smoothly controls drone movement
- **Velocity Coordinator**: Manages command priorities

## Integration Options

### Option 1: Safe Integration via Bridge (Recommended)
Use the pattern_llm_bridge to keep systems separate:

```bash
# Launch pattern nodes alongside existing system
./test_pattern_demo.sh

# Send pattern commands via separate topic
ros2 topic pub /llm/pattern_request std_msgs/msg/String "data: 'search:expanding_square'" --once
```

**Advantages:**
- No modifications to working code
- Easy to disable if issues arise
- Clean separation of concerns

### Option 2: Direct LLM Integration
Add pattern recognition to practical_llm_controller.py:

```python
# In parse_command method, add:
if "expanding square" in command_lower or "square pattern" in command_lower:
    # Publish to pattern generator
    pattern_msg = String()
    pattern_msg.data = "expanding_square:15,4"
    self.pattern_cmd_pub.publish(pattern_msg)
    
    # Start pattern execution
    control_msg = String()
    control_msg.data = "start"
    self.pattern_control_pub.publish(control_msg)
    
    return {"action": "search_pattern", "type": "expanding_square"}
```

## Testing Procedure

### 1. Test Patterns Standalone
```bash
# Generate and execute patterns manually
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:20,5'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
```

### 2. Test with Bridge
```bash
# Use LLM bridge for natural language
ros2 topic pub /llm/pattern_request std_msgs/msg/String "data: 'search:spiral'" --once
```

### 3. Monitor Execution
```bash
# Watch pattern status
ros2 topic echo /pattern_status

# View current waypoints
ros2 topic echo /pattern_waypoints
```

## Pattern Parameters

### Expanding Square
- Command: `expanding_square:SIZE,LOOPS`
- Example: `expanding_square:15,4` (15m sides, 4 expanding loops)

### Spiral
- Command: `spiral:RADIUS,TURNS`
- Example: `spiral:25,5` (25m max radius, 5 turns)

### Zigzag
- Command: `zigzag:WIDTH,HEIGHT,SPACING`
- Example: `zigzag:30,30,5` (30x30m area, 5m line spacing)

## Natural Language Examples

When integrated with LLM, these commands should work:
- "Search the area in an expanding square pattern"
- "Do a spiral search with 20 meter radius"
- "Perform a zigzag pattern over the field"
- "Stop the search pattern"
- "Pause the current pattern"

## Safety Considerations

1. **Test at Safe Altitude**: Always test patterns at >5m altitude
2. **Monitor Battery**: Patterns can cover large areas
3. **Manual Override**: Any manual command should interrupt patterns
4. **RTH Integration**: Patterns should abort if RTH is triggered

## Troubleshooting

### Pattern Not Starting
- Check if waypoints were generated: `ros2 topic echo /pattern_waypoints`
- Verify pattern executor is running
- Ensure drone is in position control mode

### Oscillation During Pattern
- Hybrid controller should handle this
- If issues persist, reduce pattern speed in pattern_executor.py

### LLM Not Recognizing Commands
- Check pattern_llm_bridge is running
- Verify topic connections
- Test with direct pattern commands first

## Next Steps

1. Run `./test_pattern_demo.sh` for full demonstration
2. Test each pattern individually
3. Verify LLM bridge works
4. Consider adding patterns to practical_llm_controller.py
5. Add vision integration (hover on detection during pattern)