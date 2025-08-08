#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TestRTHDirections(Node):
    def __init__(self):
        super().__init__('test_rth_directions')
        
        # QoS for MAVROS compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Current position
        self.current_pose = None
        self.home_position = Point()
        self.home_position.x = 0.0
        self.home_position.y = 0.0
        self.home_position.z = 3.0
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )
        
        self.cmd_sub = self.create_subscription(
            String,
            '/test_rth_cmd',
            self.command_callback,
            10
        )
        
        # Publisher - directly to MAVROS to bypass velocity coordinator
        self.vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )
        
        # Test state
        self.test_mode = None
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Test RTH Directions Node Started')
        self.get_logger().info('Commands: test_original, test_negated, test_no_transform, test_single_neg_x, test_single_neg_y, stop')
    
    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose
    
    def command_callback(self, msg: String):
        self.test_mode = msg.data
        if msg.data == 'stop':
            self.test_mode = None
            stop_vel = Twist()
            self.vel_pub.publish(stop_vel)
            self.get_logger().info('Stopped')
    
    def control_loop(self):
        if not self.test_mode or not self.current_pose:
            return
        
        # Calculate deltas
        dx = self.home_position.x - self.current_pose.position.x
        dy = self.home_position.y - self.current_pose.position.y
        dz = self.home_position.z - self.current_pose.position.z
        
        distance = (dx**2 + dy**2 + dz**2)**0.5
        
        self.get_logger().info(f'Delta: dX={dx:.2f}, dY={dy:.2f}, dZ={dz:.2f}, dist={distance:.2f}')
        
        if distance < 1.0:
            self.get_logger().info('Close to home!')
            return
        
        # Calculate normalized velocity
        vel_cmd = Twist()
        vel_cmd.linear.x = (dx / distance) * 2.0  # 2 m/s
        vel_cmd.linear.y = (dy / distance) * 2.0
        vel_cmd.linear.z = (dz / distance) * 1.0
        
        # Apply different transformations based on test mode
        transformed_vel = Twist()
        
        if self.test_mode == 'test_original':
            # Original transformation (no negation)
            transformed_vel.linear.x = vel_cmd.linear.y
            transformed_vel.linear.y = vel_cmd.linear.x
            transformed_vel.linear.z = vel_cmd.linear.z
            self.get_logger().info(f'ORIGINAL: ROS({vel_cmd.linear.x:.2f},{vel_cmd.linear.y:.2f}) → UE({transformed_vel.linear.x:.2f},{transformed_vel.linear.y:.2f})')
            
        elif self.test_mode == 'test_negated':
            # Both negated (current fix attempt)
            transformed_vel.linear.x = -vel_cmd.linear.y
            transformed_vel.linear.y = -vel_cmd.linear.x
            transformed_vel.linear.z = vel_cmd.linear.z
            self.get_logger().info(f'BOTH NEG: ROS({vel_cmd.linear.x:.2f},{vel_cmd.linear.y:.2f}) → UE({transformed_vel.linear.x:.2f},{transformed_vel.linear.y:.2f})')
            
        elif self.test_mode == 'test_no_transform':
            # No transformation
            transformed_vel = vel_cmd
            self.get_logger().info(f'NO TRANS: Sending({vel_cmd.linear.x:.2f},{vel_cmd.linear.y:.2f})')
            
        elif self.test_mode == 'test_single_neg_x':
            # Only negate X mapping
            transformed_vel.linear.x = -vel_cmd.linear.y
            transformed_vel.linear.y = vel_cmd.linear.x
            transformed_vel.linear.z = vel_cmd.linear.z
            self.get_logger().info(f'NEG X: ROS({vel_cmd.linear.x:.2f},{vel_cmd.linear.y:.2f}) → UE({transformed_vel.linear.x:.2f},{transformed_vel.linear.y:.2f})')
            
        elif self.test_mode == 'test_single_neg_y':
            # Only negate Y mapping
            transformed_vel.linear.x = vel_cmd.linear.y
            transformed_vel.linear.y = -vel_cmd.linear.x
            transformed_vel.linear.z = vel_cmd.linear.z
            self.get_logger().info(f'NEG Y: ROS({vel_cmd.linear.x:.2f},{vel_cmd.linear.y:.2f}) → UE({transformed_vel.linear.x:.2f},{transformed_vel.linear.y:.2f})')
        
        # Publish velocity
        self.vel_pub.publish(transformed_vel)

def main():
    rclpy.init()
    node = TestRTHDirections()
    rclpy.spin(node)

if __name__ == '__main__':
    main()