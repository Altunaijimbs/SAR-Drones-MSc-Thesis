# READY-TO-USE DEBUG CODE SNIPPETS

## 1. RTH Debug Logging (Copy-paste into return_to_home_node.py)

### Add after line 183 (in control_loop):
```python
        # DEBUG: Log positions and deltas
        self.get_logger().warn(f'[RTH DEBUG] Current: X={self.current_pose.position.x:.2f}, Y={self.current_pose.position.y:.2f}, Z={self.current_pose.position.z:.2f}')
        self.get_logger().warn(f'[RTH DEBUG] Home: X={self.home_position.x:.2f}, Y={self.home_position.y:.2f}, Z={self.home_position.z:.2f}')
        self.get_logger().warn(f'[RTH DEBUG] Delta: dX={dx:.2f}, dY={dy:.2f}, dZ={dz:.2f}')
        self.get_logger().warn(f'[RTH DEBUG] Distance: {distance:.2f}m')
```

### Add after line 208 (after velocity calculation):
```python
        # DEBUG: Log velocity before transform
        self.get_logger().warn(f'[RTH DEBUG] ROS Vel: X={vel_cmd.linear.x:.2f}, Y={vel_cmd.linear.y:.2f}, Z={vel_cmd.linear.z:.2f}')
```

### Add after line 214 (after transformation):
```python
        # DEBUG: Log velocity after transform
        self.get_logger().warn(f'[RTH DEBUG] UE4 Vel: X={transformed_vel.linear.x:.2f}, Y={transformed_vel.linear.y:.2f}, Z={transformed_vel.linear.z:.2f}')
```

## 2. Test Without Transformation (Replace lines 210-217):
```python
        # TEST: Skip transformation
        self.get_logger().error('[RTH TEST] SKIPPING TRANSFORMATION - SENDING ROS COORDS')
        self.vel_pub.publish(vel_cmd)  # Send ROS coordinates directly
```

## 3. Test Inverted Transformation (Replace lines 211-214):
```python
        # TEST: Inverted transformation
        transformed_vel = Twist()
        transformed_vel.linear.x = vel_cmd.linear.y   # No negation
        transformed_vel.linear.y = -vel_cmd.linear.x  # Negate X instead
        transformed_vel.linear.z = vel_cmd.linear.z
        self.get_logger().error('[RTH TEST] USING INVERTED TRANSFORMATION')
```

## 4. Add Position Echo Node (New file: position_monitor.py):
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PositionMonitor(Node):
    def __init__(self):
        super().__init__('position_monitor')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos
        )
        
        self.vel_sub = self.create_subscription(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            self.vel_callback,
            10
        )
        
        self.timer = self.create_timer(1.0, self.log_status)
        self.current_pos = None
        self.current_vel = None
        
    def pose_callback(self, msg):
        self.current_pos = msg.pose.position
        
    def vel_callback(self, msg):
        self.current_vel = msg.linear
        
    def log_status(self):
        if self.current_pos and self.current_vel:
            self.get_logger().info(
                f'POS: ({self.current_pos.x:.1f}, {self.current_pos.y:.1f}, {self.current_pos.z:.1f}) | '
                f'VEL: ({self.current_vel.x:.1f}, {self.current_vel.y:.1f}, {self.current_vel.z:.1f})'
            )

def main():
    rclpy.init()
    node = PositionMonitor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## 5. Manual Direction Test Commands:
```bash
# Test each direction for 2 seconds
echo "Testing FORWARD (ROS +X)..."
ros2 topic pub /rth/velocity_command geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}}" --once

echo "Testing RIGHT (ROS +Y)..."  
ros2 topic pub /rth/velocity_command geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 1.0, z: 0.0}}" --once

echo "Testing UP (ROS +Z)..."
ros2 topic pub /rth/velocity_command geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 1.0}}" --once
```

## 6. Velocity Coordinator Status Monitor:
```bash
# Run in separate terminal during RTH test
watch -n 0.5 'ros2 topic echo /velocity_coordinator/active_source --once'
```

## 7. Compare Grid Search vs RTH Velocities:
```bash
# Terminal 1: Start grid search and capture velocities
ros2 topic echo /search_pattern/velocity_command > grid_velocities.txt &
ros2 topic pub /search_command std_msgs/msg/String "data: 'search'" --once

# Terminal 2: Start RTH and capture velocities  
ros2 topic echo /rth/velocity_command > rth_velocities.txt &
ros2 topic pub /stop_command std_msgs/msg/String "data: 'stop'" --once

# Compare the outputs
```

## 8. Emergency Debug Launch (Minimal nodes):
```bash
# Just the essentials for coordinate testing
pkill -f ros2; pkill -f px4

# T1: PX4
cd ~/PX4-Autopilot && make px4_sitl none_iris

# T2: Play UE5

# T3: AirSim
cd /home/mbs/Desktop/airsim/Cosys-AirSim/ros2 && source install/setup.bash && ros2 launch airsim_ros_pkgs airsim_node.launch.py

# T4: MAVROS  
ros2 run mavros mavros_node --ros-args --param fcu_url:="udp://:14550@127.0.0.1:14540"

# T5: Direct velocity test
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws && source install/setup.bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Now send raw velocity commands and observe
```

## 9. PX4 Direct Test (bypass ROS):
```bash
# In PX4 console after connection
commander takeoff
commander mode offboard

# Test directions in PX4 coordinates
# This tells us what PX4 expects
```

## 10. Quick Rebuild Commands:
```bash
# After any code change
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
colcon build --packages-select search_patterns && source install/setup.bash

# Restart just RTH node
pkill -f return_to_home
ros2 run search_patterns return_to_home
```