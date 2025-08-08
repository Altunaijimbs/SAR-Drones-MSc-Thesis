# AS2 Integration Plan for SAR Search Patterns

## Components to Adapt:

### 1. Trajectory Generation (PRIORITY 1)
**Source**: `as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior`
- Use polynomial trajectory generation for smooth curves
- Perfect for spiral and expanding square patterns
- Already handles waypoint interpolation

### 2. Path Following (PRIORITY 2)
**Source**: `as2_behaviors_motion/follow_path_behavior`
- Professional waypoint following implementation
- Supports both position and trajectory modes
- Could replace our basic waypoint following

### 3. Motion Controller Improvements
**Source**: `as2_motion_controller/plugins/pid_speed_controller`
- More sophisticated than our current implementation
- Includes proper gain scheduling and mode management

## Implementation Steps:

### Step 1: Study AS2 Trajectory Generation
1. Extract polynomial trajectory math from AS2
2. Create simplified version for our search patterns
3. Generate smooth waypoints for:
   - Expanding square (using polynomial corners)
   - Spiral (using parametric equations)
   - Zigzag (with smooth turns)

### Step 2: Adapt Path Following
1. Study AS2's follow_path_behavior structure
2. Create simplified version that works with our hybrid controller
3. Add to our practical LLM for pattern execution

### Step 3: Create Pattern Generator Node
```python
class SearchPatternGenerator(Node):
    def generate_expanding_square(self, size, expansions):
        # Use AS2 polynomial trajectory math
        pass
    
    def generate_spiral(self, radius, spacing):
        # Use parametric spiral with smooth interpolation
        pass
    
    def generate_zigzag(self, width, length, spacing):
        # Generate with rounded corners
        pass
```

## Key AS2 Concepts to Borrow:

1. **Behavior Server Pattern** - Clean action server implementation
2. **Plugin Architecture** - Swappable implementations
3. **Trajectory Sampling** - Smooth interpolation between waypoints
4. **Transform Management** - Handles map->odom transforms properly

## Files to Study:

1. `/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp`
2. `/as2_behaviors_motion/follow_path_behavior/src/follow_path_behavior.cpp`
3. `/as2_motion_controller/plugins/pid_speed_controller/src/pid_speed_controller.cpp`

## Advantages of AS2 Integration:

- Professional, tested code
- Smooth trajectories (no jerky movements)
- Proper error handling and recovery
- Extensible plugin architecture
- Better performance than basic implementations