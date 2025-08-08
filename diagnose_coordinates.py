#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String

class CoordinateDiagnostic(Node):
    def __init__(self):
        super().__init__('coordinate_diagnostic')
        
        # Subscribe to all velocity sources
        self.subs = []
        self.subs.append(self.create_subscription(
            Twist, '/keepalive/velocity_command', 
            lambda m: self.vel_callback(m, 'KeepAlive'), 10))
        self.subs.append(self.create_subscription(
            Twist, '/rth/velocity_command', 
            lambda m: self.vel_callback(m, 'RTH'), 10))
        self.subs.append(self.create_subscription(
            Twist, '/search_pattern/velocity_command', 
            lambda m: self.vel_callback(m, 'Search'), 10))
        self.subs.append(self.create_subscription(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 
            lambda m: self.vel_callback(m, 'MAVROS'), 10))
        
        # Position
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.pose_callback, 10)
        
        self.current_pose = None
        self.get_logger().info('Coordinate diagnostic started')
        
    def vel_callback(self, msg: Twist, source: str):
        self.get_logger().info(
            f'[{source}] Velocity: X={msg.linear.x:.3f}, Y={msg.linear.y:.3f}, Z={msg.linear.z:.3f}'
        )
        
    def pose_callback(self, msg: PoseStamped):
        if self.current_pose is None:
            self.get_logger().info(
                f'[Position] Initial: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}, Z={msg.pose.position.z:.2f}'
            )
        self.current_pose = msg.pose

def main():
    rclpy.init()
    node = CoordinateDiagnostic()
    rclpy.spin(node)

if __name__ == '__main__':
    main()