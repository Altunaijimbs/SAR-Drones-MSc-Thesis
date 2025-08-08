#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
from mavros_msgs.msg import State
from std_msgs.msg import String, Bool
import time
import threading

class ComprehensiveRTHTest(Node):
    def __init__(self):
        super().__init__('comprehensive_rth_test')
        
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
        
        # Subscribers to monitor all RTH-related topics
        self.rth_vel_sub = self.create_subscription(
            Twist,
            '/rth/velocity_command',
            self.rth_velocity_callback,
            10
        )
        
        self.coord_vel_sub = self.create_subscription(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            self.coordinator_velocity_callback,
            10
        )
        
        self.coord_status_sub = self.create_subscription(
            String,
            '/velocity_coordinator/active_source',
            self.coordinator_status_callback,
            10
        )
        
        # Current position for simulation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 3.0
        
        self.get_logger().info('Comprehensive RTH Test Node initialized')
        
        # Start test sequence in a separate thread
        self.test_thread = threading.Thread(target=self.run_test_sequence)
        self.test_thread.daemon = True
        self.test_thread.start()
    
    def rth_velocity_callback(self, msg):
        """Monitor RTH velocity commands"""
        self.get_logger().error(f'[RTH OUTPUT] RTH Velocity: X={msg.linear.x:.2f}, Y={msg.linear.y:.2f}, Z={msg.linear.z:.2f}')
    
    def coordinator_velocity_callback(self, msg):
        """Monitor final velocity commands from coordinator"""
        self.get_logger().info(f'[FINAL VEL] Coordinator Output: X={msg.linear.x:.2f}, Y={msg.linear.y:.2f}, Z={msg.linear.z:.2f}')
    
    def coordinator_status_callback(self, msg):
        """Monitor which source is active in coordinator"""
        self.get_logger().warn(f'[COORD STATUS] {msg.data}')
    
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
        self.current_x, self.current_y, self.current_z = x, y, z
    
    def publish_state(self, armed, mode):
        """Publish drone state"""
        state_msg = State()
        state_msg.connected = True
        state_msg.armed = armed
        state_msg.guided = True
        state_msg.mode = mode
        
        self.state_pub.publish(state_msg)
    
    def run_test_sequence(self):
        """Execute the complete test sequence"""
        time.sleep(3)  # Wait for connections
        
        self.get_logger().error('========== COMPREHENSIVE RTH TEST ==========')
        
        # Step 1: Set initial position and arm
        self.get_logger().error('STEP 1: Setting initial position and arming')
        self.publish_pose(0.0, 0.0, 3.0)
        time.sleep(0.5)
        self.publish_state(True, 'OFFBOARD')
        time.sleep(2)
        
        # Step 2: Manual home set
        self.get_logger().error('STEP 2: Setting home position manually')
        home_cmd = String()
        home_cmd.data = 'set'
        self.set_home_pub.publish(home_cmd)
        time.sleep(1)
        
        # Step 3: Move drone away
        self.get_logger().error('STEP 3: Moving drone away from home')
        positions = [
            (2.0, 1.0, 3.5),
            (4.0, 2.0, 4.0),
            (6.0, 4.0, 4.5),
            (8.0, 6.0, 5.0)
        ]
        
        for x, y, z in positions:
            self.get_logger().warn(f'Moving to position: ({x:.1f}, {y:.1f}, {z:.1f})')
            self.publish_pose(x, y, z)
            time.sleep(1)
        
        # Step 4: Trigger RTH
        self.get_logger().error('STEP 4: TRIGGERING RETURN TO HOME!')
        rth_cmd = String()
        rth_cmd.data = 'return home'
        self.rth_cmd_pub.publish(rth_cmd)
        time.sleep(0.5)
        
        # Step 5: Monitor RTH for extended period
        self.get_logger().error('STEP 5: Monitoring RTH behavior...')
        for i in range(20):  # 10 seconds of monitoring
            # Keep publishing current position for RTH calculations
            self.publish_pose(self.current_x, self.current_y, self.current_z)
            time.sleep(0.5)
            
            if i % 4 == 0:  # Every 2 seconds
                self.get_logger().warn(f'RTH Active - Current position: ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f})')
        
        self.get_logger().error('========== RTH TEST COMPLETE ==========')

def main():
    rclpy.init()
    node = ComprehensiveRTHTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()