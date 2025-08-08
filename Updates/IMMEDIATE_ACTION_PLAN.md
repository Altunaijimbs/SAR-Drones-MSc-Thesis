# Immediate Action Plan - Coordinate System Fix
## Priority: CRITICAL
## Time Required: 2-3 hours

---

## Step 1: Establish Ground Truth (30 minutes)

### Test A: Raw MAVROS Velocity Test
```bash
# Terminal 1: Start minimal system
./test_fixed_system.sh

# Terminal 2: Monitor actual movement
ros2 run search_patterns diagnose_coordinates

# Terminal 3: Send raw velocities and observe
# Test X-axis (should move forward/back in ROS frame)
ros2 topic pub /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}}" --once
# Wait 2 seconds, observe direction

# Test Y-axis (should move left/right in ROS frame)  
ros2 topic pub /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 1.0, z: 0.0}}" --once
# Wait 2 seconds, observe direction

# Test Z-axis (should move up/down)
ros2 topic pub /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 1.0}}" --once
# Wait 2 seconds, observe direction
```

**Record Results:**
- MAVROS X=1.0 → Drone moves: _______
- MAVROS Y=1.0 → Drone moves: _______
- MAVROS Z=1.0 → Drone moves: _______

### Test B: Smart Keep-Alive Commands
```bash
# Test each direction with smart keep-alive
ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once
# Observe: Direction _______ , MAVROS velocity (X,Y,Z) = _______

ros2 topic pub /simple_command std_msgs/msg/String "data: 'right'" --once
# Observe: Direction _______ , MAVROS velocity (X,Y,Z) = _______

ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once
# Observe: Direction _______ , MAVROS velocity (X,Y,Z) = _______
```

---

## Step 2: Create Transformation Test Node (30 minutes)

Create `/home/mbs/SAR-Drones-MSc-Thesis/coordinate_test.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CoordinateTest(Node):
    def __init__(self):
        super().__init__('coordinate_test')
        
        self.vel_pub = self.create_publisher(
            Twist, 
            '/mavros/setpoint_velocity/cmd_vel_unstamped', 
            10
        )
        
        self.cmd_sub = self.create_subscription(
            String,
            '/test_command',
            self.command_callback,
            10
        )
        
        self.get_logger().info('Coordinate test node ready')
        self.get_logger().info('Commands: no_transform, transform_current, transform_negated')
    
    def command_callback(self, msg):
        if msg.data == 'no_transform':
            self.test_no_transform()
        elif msg.data == 'transform_current':
            self.test_current_transform()
        elif msg.data == 'transform_negated':
            self.test_negated_transform()
    
    def test_no_transform(self):
        # Send ROS coordinates directly
        vel = Twist()
        vel.linear.x = 1.0  # ROS forward
        self.vel_pub.publish(vel)
        self.get_logger().warn('NO TRANSFORM: Sent ROS X=1.0')
    
    def test_current_transform(self):
        # Current transformation (from code)
        ros_vel = Twist()
        ros_vel.linear.x = 1.0  # ROS forward
        
        ue_vel = Twist()
        ue_vel.linear.x = ros_vel.linear.y  # Y→X
        ue_vel.linear.y = ros_vel.linear.x  # X→Y
        
        self.vel_pub.publish(ue_vel)
        self.get_logger().warn(f'CURRENT TRANSFORM: ROS X=1.0 → UE X={ue_vel.linear.x}, Y={ue_vel.linear.y}')
    
    def test_negated_transform(self):
        # Test with negation
        ros_vel = Twist()
        ros_vel.linear.x = 1.0  # ROS forward
        
        ue_vel = Twist()
        ue_vel.linear.x = -ros_vel.linear.y  # -Y→X
        ue_vel.linear.y = ros_vel.linear.x   # X→Y
        
        self.vel_pub.publish(ue_vel)
        self.get_logger().warn(f'NEGATED TRANSFORM: ROS X=1.0 → UE X={ue_vel.linear.x}, Y={ue_vel.linear.y}')

def main():
    rclpy.init()
    node = CoordinateTest()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

Run tests:
```bash
python3 coordinate_test.py

# In another terminal:
ros2 topic pub /test_command std_msgs/msg/String "data: 'no_transform'" --once
# Observe movement direction

ros2 topic pub /test_command std_msgs/msg/String "data: 'transform_current'" --once  
# Observe movement direction

ros2 topic pub /test_command std_msgs/msg/String "data: 'transform_negated'" --once
# Observe movement direction
```

---

## Step 3: Fix Based on Results (1 hour)

### If drone moves incorrectly with current transform:

**Option A: Revert Smart Keep-Alive** (if it was originally correct)
```python
# In smart_keep_alive_node.py, change back:
elif command == "left":
    velocity.linear.x = 0.5   # Positive for left
elif command == "right":  
    velocity.linear.x = -0.5  # Negative for right
```

**Option B: Fix Transformation** (if transformation is wrong)
```python
# In return_to_home_node.py, grid_search.py, etc:
transformed_vel.linear.x = -vel_cmd.linear.y  # Add negation back
transformed_vel.linear.y = vel_cmd.linear.x   
```

**Option C: No Transformation** (if MAVROS already handles it)
```python
# Just pass through:
self.vel_pub.publish(vel_cmd)  # No transformation
```

---

## Step 4: Validate RTH (30 minutes)

Once correct transformation is found:

```bash
# Run comprehensive RTH test
python3 /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/comprehensive_rth_test.py

# In another terminal, monitor:
ros2 topic echo /rth/velocity_command
ros2 topic echo /mavros/local_position/pose
```

Success Criteria:
- Drone moves toward home position
- Velocity vectors point in correct direction
- Distance to home decreases

---

## Step 5: Document Solution (15 minutes)

Create `/home/mbs/SAR-Drones-MSc-Thesis/Updates/COORDINATE_FIX_RESULTS.md`:

```markdown
# Coordinate System Fix Results
Date: August 3, 2025

## Ground Truth Results
- MAVROS X=1.0 moves drone: [FORWARD/BACKWARD/LEFT/RIGHT]
- MAVROS Y=1.0 moves drone: [FORWARD/BACKWARD/LEFT/RIGHT]
- MAVROS Z=1.0 moves drone: [UP/DOWN]

## Correct Transformation
[Document the working transformation]

## Files Updated
1. return_to_home_node.py - Line XXX
2. grid_search.py - Line XXX
3. smart_keep_alive_node.py - Line XXX (if needed)

## Validation
- RTH tested: [WORKING/STILL BROKEN]
- Grid search tested: [WORKING/STILL BROKEN]
```

---

## If Still Stuck After This:

1. **Check for Double Transformation**:
   - Is velocity coordinator transforming again?
   - Is MAVROS doing internal transformation?

2. **Test Position Coordinates**:
   - RTH uses position for calculations
   - Position frame might differ from velocity frame

3. **Nuclear Option**:
   - Revert to commit before coordinate "fixes"
   - Test if original system worked

---

## Remember:
- **One change at a time**
- **Test after each change**
- **Document what works**
- **Don't guess - verify!**

Time estimate: 2-3 hours to definitively solve coordinate issue