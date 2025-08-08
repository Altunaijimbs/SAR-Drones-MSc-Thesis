#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from drone_interfaces.msg import ObstacleArray, Obstacle
from std_msgs.msg import Header
import numpy as np
import math
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Vector3


class LidarObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_avoidance_node')
        
        # Parameters
        self.declare_parameter('min_distance', 2.0)  # meters
        self.declare_parameter('sector_angle', 15.0)  # degrees per sector
        self.declare_parameter('max_range', 20.0)  # meters (from settings.json)
        self.declare_parameter('height_threshold', 0.5)  # meters above/below drone
        
        # Get parameters
        self.min_distance = self.get_parameter('min_distance').value
        self.sector_angle = self.get_parameter('sector_angle').value
        self.max_range = self.get_parameter('max_range').value
        self.height_threshold = self.get_parameter('height_threshold').value
        
        # Calculate number of sectors (360 degrees coverage)
        self.num_sectors = int(360 / self.sector_angle)
        self.sector_distances = [self.max_range] * self.num_sectors
        
        # Subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/airsim_node/PX4/LidarSensor1/point_cloud',
            self.lidar_callback,
            10
        )
        
        # Publishers
        self.lidar_obstacles_pub = self.create_publisher(
            ObstacleArray,
            '/drone/lidar_obstacles',
            10
        )
        
        self.get_logger().info(f'LiDAR Obstacle Avoidance initialized with {self.num_sectors} sectors')
    
    def lidar_callback(self, msg: PointCloud2):
        """Process LiDAR point cloud data"""
        # Reset sector distances
        self.sector_distances = [self.max_range] * self.num_sectors
        
        # Convert PointCloud2 to numpy array
        points = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        
        if not points:
            return
        
        points = np.array(points)
        
        # Process points and find closest obstacles in each sector
        obstacles = self.process_lidar_points(points)
        
        # Publish obstacles
        obstacle_msg = ObstacleArray()
        obstacle_msg.header.stamp = msg.header.stamp
        obstacle_msg.obstacles = obstacles
        self.lidar_obstacles_pub.publish(obstacle_msg)
    
    def process_lidar_points(self, points: np.ndarray) -> list:
        """Process LiDAR points and detect obstacles"""
        obstacles = []
        
        # Transform from AirSim/UE4 to ROS coordinates for processing
        # AirSim/UE4: X-right, Y-forward, Z-up
        # ROS: X-forward, Y-left, Z-up
        
        for point in points:
            # Original point in AirSim/UE4 coordinates
            ue_x, ue_y, ue_z = point
            
            # Transform to ROS for processing
            ros_x = ue_y  # UE4 Y (forward) -> ROS X
            ros_y = -ue_x  # UE4 X (right) -> ROS -Y (left)
            ros_z = ue_z  # Same
            
            # Filter points by height (only consider obstacles near drone level)
            if abs(ros_z) > self.height_threshold:
                continue
            
            # Calculate distance and angle in ROS frame
            distance = math.sqrt(ros_x**2 + ros_y**2)
            
            if distance < 0.1 or distance > self.max_range:
                continue
            
            # Calculate angle (0 degrees is forward in ROS)
            angle = math.degrees(math.atan2(ros_y, ros_x))
            angle = (angle + 360) % 360  # Normalize to 0-360
            
            # Determine sector
            sector = int(angle / self.sector_angle)
            sector = min(sector, self.num_sectors - 1)
            
            # Update minimum distance for this sector
            if distance < self.sector_distances[sector]:
                self.sector_distances[sector] = distance
        
        # Create obstacles for sectors with close objects
        for sector in range(self.num_sectors):
            if self.sector_distances[sector] < self.min_distance:
                obstacle = self.create_obstacle_from_sector(sector, self.sector_distances[sector])
                obstacles.append(obstacle)
        
        return obstacles
    
    def create_obstacle_from_sector(self, sector: int, distance: float) -> Obstacle:
        """Create an obstacle message from sector data"""
        obstacle = Obstacle()
        
        # Calculate angle for this sector (center of sector)
        angle = (sector * self.sector_angle + self.sector_angle / 2) * math.pi / 180
        
        # Calculate position in ROS coordinates
        ros_x = distance * math.cos(angle)
        ros_y = distance * math.sin(angle)
        
        # Transform back to UE4 coordinates for consistency with vision system
        obstacle.position.x = -ros_y  # ROS Y -> UE4 -X
        obstacle.position.y = ros_x   # ROS X -> UE4 Y
        obstacle.position.z = 0.0
        
        # Set obstacle properties
        obstacle.type = "lidar_obstacle"
        obstacle.confidence = 0.9
        
        # Estimate dimensions (conservative estimate)
        obstacle.dimensions = Vector3()
        obstacle.dimensions.x = 1.0
        obstacle.dimensions.y = 1.0
        obstacle.dimensions.z = 2.0
        
        return obstacle
    
    def get_safe_direction(self) -> tuple:
        """Find the safest direction to move based on LiDAR data"""
        # Find sector with maximum distance
        max_distance = max(self.sector_distances)
        safest_sector = self.sector_distances.index(max_distance)
        
        # Calculate angle for safest direction
        safe_angle = (safest_sector * self.sector_angle + self.sector_angle / 2) * math.pi / 180
        
        # Convert to velocity commands
        if max_distance > self.min_distance:
            # Safe to move in this direction
            ros_vx = math.cos(safe_angle)
            ros_vy = math.sin(safe_angle)
            
            # Transform to UE4 velocity commands
            ue_vx = -ros_vy  # ROS Y -> UE4 -X
            ue_vy = ros_vx   # ROS X -> UE4 Y
            
            return ue_vx, ue_vy, max_distance
        else:
            # No safe direction - stop
            return 0.0, 0.0, 0.0


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()