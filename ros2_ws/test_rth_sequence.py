#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
from mavros_msgs.msg import State
from std_msgs.msg import String, Bool
import time

class RTHTestNode(Node):
    def __init__(self):
        super().__init__('rth_test_node')
        
        # Publishers to simulate drone data
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/local_position/pose',
            10
        )
        
        self.state_pub = self.create_publisher(
            State,
            '/mavros/state',
            10
        )
        
        self.set_home_pub = self.create_publisher(
            String,
            '/set_home_position',
            10
        )
        
        self.rth_cmd_pub = self.create_publisher(
            String,
            '/rth_command',
            10
        )
        
        # Subscriber to monitor RTH velocity commands
        self.rth_vel_sub = self.create_subscription(
            Twist,
            '/rth/velocity_command',
            self.rth_velocity_callback,
            10
        )
        
        self.get_logger().info('RTH Test Node initialized')
        
        # Wait a moment for connections
        time.sleep(2)
        
        # Start the test sequence
        self.test_sequence()
    
    def rth_velocity_callback(self, msg):
        """Monitor RTH velocity commands"""
        self.get_logger().info(f'RTH Velocity Command: X={msg.linear.x:.2f}, Y={msg.linear.y:.2f}, Z={msg.linear.z:.2f}')
    
    def publish_pose(self, x, y, z):
        """Publish drone position"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0
        
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f'Published pose: X={x:.2f}, Y={y:.2f}, Z={z:.2f}')
    
    def publish_state(self, armed, mode):
        """Publish drone state"""
        state_msg = State()
        state_msg.connected = True
        state_msg.armed = armed
        state_msg.guided = True
        state_msg.mode = mode
        
        self.state_pub.publish(state_msg)
        self.get_logger().info(f'Published state: Armed={armed}, Mode={mode}')
    
    def test_sequence(self):
        """Execute the complete test sequence"""
        self.get_logger().warn('========== STARTING RTH TEST SEQUENCE ==========')
        
        # Step 1: Simulate takeoff position (home)
        self.get_logger().warn('Step 1: Setting takeoff position (3m altitude)')
        self.publish_pose(0.0, 0.0, 3.0)
        time.sleep(1)
        
        # Step 2: Arm drone and set to OFFBOARD mode
        self.get_logger().warn('Step 2: Arming drone and setting OFFBOARD mode')
        self.publish_state(True, 'OFFBOARD')
        time.sleep(2)  # Give time for auto home position saving
        
        # Step 3: Manually set home position (as backup)
        self.get_logger().warn('Step 3: Manually setting home position')
        home_cmd = String()
        home_cmd.data = 'set'
        self.set_home_pub.publish(home_cmd)
        time.sleep(1)
        
        # Step 4: Move drone away from home
        self.get_logger().warn('Step 4: Moving drone away from home position')
        self.publish_pose(5.0, 3.0, 4.0)  # Move to (5, 3, 4)
        time.sleep(2)
        
        self.publish_pose(8.0, 6.0, 5.0)  # Move further to (8, 6, 5)
        time.sleep(2)
        
        # Step 5: Trigger RTH
        self.get_logger().warn('Step 5: Triggering Return to Home')
        rth_cmd = String()
        rth_cmd.data = 'return home'
        self.rth_cmd_pub.publish(rth_cmd)
        
        # Step 6: Monitor RTH for 10 seconds
        self.get_logger().warn('Step 6: Monitoring RTH behavior for 10 seconds...')
        start_time = time.time()
        while time.time() - start_time < 10:
            # Continue publishing current position for RTH calculations
            self.publish_pose(8.0, 6.0, 5.0)
            time.sleep(0.5)
        
        self.get_logger().warn('========== RTH TEST SEQUENCE COMPLETE ==========')

def main():
    rclpy.init()
    node = RTHTestNode()
    
    # Keep the node alive for monitoring
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()