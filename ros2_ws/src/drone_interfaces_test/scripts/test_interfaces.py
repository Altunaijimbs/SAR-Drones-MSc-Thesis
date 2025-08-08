#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from drone_interfaces.msg import (
    DroneState, Waypoint, WaypointList, 
    MissionStatus, SceneDescription, LLMCommand
)
from drone_interfaces.srv import SetMission, ArmDrone
from geometry_msgs.msg import Point, Pose, Twist
from std_msgs.msg import Header

class InterfaceTestNode(Node):
    def __init__(self):
        super().__init__('interface_test_node')
        
        # Test publishers
        self.drone_state_pub = self.create_publisher(DroneState, '/test/drone_state', 10)
        self.mission_status_pub = self.create_publisher(MissionStatus, '/test/mission_status', 10)
        
        # Test timer
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Interface test node started!')
        
    def timer_callback(self):
        # Test DroneState message
        drone_state = DroneState()
        drone_state.header = Header()
        drone_state.header.stamp = self.get_clock().now().to_msg()
        drone_state.drone_id = "test_drone_1"
        drone_state.mode = "GUIDED"
        drone_state.armed = True
        drone_state.battery_percentage = 85.5
        
        self.drone_state_pub.publish(drone_state)
        
        # Test MissionStatus message
        mission_status = MissionStatus()
        mission_status.header = Header()
        mission_status.header.stamp = self.get_clock().now().to_msg()
        mission_status.mission_id = "test_mission_001"
        mission_status.status = "EXECUTING"
        mission_status.total_waypoints = 5
        mission_status.current_waypoint_index = 2
        mission_status.progress_percentage = 40.0
        
        self.mission_status_pub.publish(mission_status)
        
        self.get_logger().info('Published test messages')

def main(args=None):
    rclpy.init(args=args)
    node = InterfaceTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
