# SAR Drone System - Session 2 Progress Report
## Date: August 3, 2025 (Afternoon Session)
## Focus: Coordinate System Debugging and System Overhaul

---

## Session Summary

This session focused on diagnosing and fixing the Return to Home (RTH) navigation issues through systematic testing and a complete system overhaul.

---

## Major Findings

### 1. Coordinate System Test Results

**Manual Testing Revealed:**
- Smart keep-alive node had X-axis inverted:
  - "right" command (x = -0.5) → drone moved LEFT
  - "left" command (x = 0.5) → drone moved RIGHT

**Actual UE4/AirSim Coordinates (via MAVROS):**
- X-axis: Positive = RIGHT, Negative = LEFT (standard)
- Y-axis: Positive = FORWARD, Negative = BACKWARD
- Z-axis: Positive = UP, Negative = DOWN

### 2. System Architecture Issues Discovered

Through comprehensive analysis of all nodes, we identified:

#### **Multiple Velocity Control Systems:**
- `VelocityCoordinator` and `VelocityMultiplexer` both trying to arbitrate commands
- `keep_alive_node.py` bypassing coordinator by publishing directly to MAVROS
- Result: Command conflicts and race conditions

#### **Coordinate Transformation Inconsistencies:**
- Grid search: Used negation in X-axis transformation
- RTH: Used same negation
- Smart keep-alive: Had inverted X-axis commands
- Result: Two wrongs made a right (accidentally worked)

#### **Command Topic Conflicts:**
- `/stop_command` interpreted as:
  - RTH node: "trigger return to home"
  - User expectation: "stop movement"
- Result: Confusion and unexpected behavior

---

## Actions Taken

### 1. System Overhaul

**Disabled Conflicting Nodes:**
```bash
- keep_alive_node.py → keep_alive_node.py.DISABLED
- velocity_multiplexer.py → velocity_multiplexer.py.DISABLED  
- obstacle_avoidance.py → obstacle_avoidance.py.DISABLED
```

**Fixed Coordinate Transformations:**
- Removed negation from all transformations
- Updated all nodes to use consistent mapping:
  ```python
  transformed_vel.linear.x = vel_cmd.linear.y  # ROS Y → UE X (no negation)
  transformed_vel.linear.y = vel_cmd.linear.x  # ROS X → UE Y
  transformed_vel.linear.z = vel_cmd.linear.z  # ROS Z → UE Z
  ```

**Fixed Smart Keep-Alive X-axis:**
```python
# Before (WRONG):
"left" → x = 0.5
"right" → x = -0.5

# After (CORRECT):
"left" → x = -0.5
"right" → x = 0.5
```

**Renamed Conflicting Topics:**
- `/stop_command` → `/rth_command` for RTH triggering

### 2. Created Test Infrastructure

- `test_fixed_system.sh`: Clean launch script with proper node order
- `diagnose_coordinates.py`: Diagnostic tool for velocity monitoring

---

## Remaining Issues

### 1. RTH Still Not Working Correctly
- After fixes, RTH now drifts RIGHT instead of LEFT
- Suggests possible issue with position coordinate handling vs velocity coordinates
- May be related to double-correction (fixed both the source and compensation)

### 2. PX4 Preflight Check Failures
- PX4 becomes "unhappy" after running launch scripts
- Refuses arming with preflight check failures
- Likely caused by:
  - Position estimator confusion when using backspace in AirSim
  - Rapid node initialization causing parameter conflicts
  - Velocity command timeouts

### 3. Altitude Control Issues
- When "stop" command sent, drone returns to default takeoff altitude
- Smart keep-alive sets Z velocity to 0, which PX4 interprets as "maintain default altitude"

---

## Key Insights

### The Core Problem
The system had evolved with compensating errors:
1. Smart keep-alive had wrong X-axis
2. Grid search compensated with negated transformation
3. When we fixed both, the system broke in opposite direction

### Coordinate System Complexity
- Position coordinates from MAVROS
- Velocity commands to MAVROS
- Both need transformation but possibly differently
- Backspace in AirSim only resets visual position, not PX4's estimator

---

## Next Steps Required

1. **Determine Correct Coordinate System:**
   - Test each component in isolation
   - Document exact transformations needed for:
     - Position reading from MAVROS
     - Velocity sending to MAVROS
     - Scene understanding from AirSim

2. **Fix PX4 Integration:**
   - Add proper reset mechanism after AirSim position reset
   - Implement velocity command continuity checking
   - Consider using position control instead of velocity for RTH

3. **Systematic Testing Protocol:**
   - Test each node individually before integration
   - Create coordinate system test suite
   - Document expected vs actual behavior

4. **Consider Alternative Approaches:**
   - Use GPS coordinates for RTH instead of local position
   - Implement position-based control for more accurate navigation
   - Add visual feedback for debugging (publish markers)

---

## Files Modified Today

1. `/search_patterns/return_to_home_node.py`:
   - Fixed coordinate transformation
   - Added debug logging
   - Changed topic from `/stop_command` to `/rth_command`

2. `/search_patterns/fixed_grid_search.py`:
   - Fixed coordinate transformation

3. `/search_patterns/grid_search.py`:
   - Fixed coordinate transformation

4. `/llm_controller/smart_keep_alive_node.py`:
   - Fixed X-axis inversion

5. **Disabled Files:**
   - `keep_alive_node.py`
   - `velocity_multiplexer.py`
   - `obstacle_avoidance.py`

---

## Lessons Learned

1. **Don't Trust Working Code**: The grid search "worked" but only because it compensated for another bug
2. **Test in Isolation**: Each component should be tested independently before integration
3. **Document Coordinate Systems**: Critical to have clear documentation of expected coordinate frames
4. **Avoid Compensating Fixes**: Fix the root cause, not symptoms

---

## Time Spent

- Coordinate system testing: 1 hour
- System analysis: 45 minutes  
- Code fixes and overhaul: 1 hour
- Testing and debugging: 30 minutes
- Total: ~3.25 hours

---

*Report compiled: August 3, 2025, Afternoon Session*
*Next session should focus on isolated component testing and establishing ground truth for coordinate systems*