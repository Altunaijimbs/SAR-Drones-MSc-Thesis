#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from drone_interfaces.msg import ObstacleArray
from std_msgs.msg import Bool
import math
from enum import Enum
import time


class AvoidanceMode(Enum):
    NONE = 0
    SLOW_DOWN = 1
    LATERAL_AVOID = 2
    VERTICAL_AVOID = 3
    EMERGENCY_STOP = 4


class FusionObstacleAvoidanceNode(Node):
    """
    Fuses vision-based and LiDAR-based obstacle detection for robust avoidance
    """
    
    def __init__(self):
        super().__init__('fusion_obstacle_avoidance_node')
        
        # Parameters
        self.declare_parameter('safety_distance', 3.0)
        self.declare_parameter('critical_distance', 1.5)
        self.declare_parameter('lidar_weight', 0.6)  # Weight for LiDAR vs vision
        self.declare_parameter('avoidance_speed', 1.0)
        
        # Get parameters
        self.safety_distance = self.get_parameter('safety_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.lidar_weight = self.get_parameter('lidar_weight').value
        self.avoidance_speed = self.get_parameter('avoidance_speed').value
        
        # Subscribers
        self.vision_obstacles_sub = self.create_subscription(
            ObstacleArray,
            '/drone/obstacles',
            self.vision_obstacles_callback,
            10
        )
        
        self.lidar_obstacles_sub = self.create_subscription(
            ObstacleArray,
            '/drone/lidar_obstacles',
            self.lidar_obstacles_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers
        self.safe_vel_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped_safe',
            10
        )
        
        self.avoidance_status_pub = self.create_publisher(
            Bool,
            '/drone/obstacle_avoidance_active',
            10
        )
        
        # State
        self.vision_obstacles = []
        self.lidar_obstacles = []
        self.requested_velocity = Twist()
        self.avoidance_active = False
        self.last_vision_time = 0
        self.last_lidar_time = 0
        
        # Control timer at 20Hz
        self.control_timer = self.create_timer(0.05, self.fusion_control_loop)
        
        self.get_logger().info('Fusion Obstacle Avoidance initialized')
    
    def vision_obstacles_callback(self, msg: ObstacleArray):
        """Update vision-based obstacles"""
        self.vision_obstacles = msg.obstacles
        self.last_vision_time = time.time()
    
    def lidar_obstacles_callback(self, msg: ObstacleArray):
        """Update LiDAR-based obstacles"""
        self.lidar_obstacles = msg.obstacles
        self.last_lidar_time = time.time()
    
    def cmd_vel_callback(self, msg: Twist):
        """Store requested velocity"""
        self.requested_velocity = msg
    
    def fusion_control_loop(self):
        """Main control loop with sensor fusion"""
        # Check sensor timeouts (1 second)
        current_time = time.time()
        vision_valid = (current_time - self.last_vision_time) < 1.0
        lidar_valid = (current_time - self.last_lidar_time) < 1.0
        
        # Fuse obstacles from both sources
        fused_obstacles = self.fuse_obstacles(vision_valid, lidar_valid)
        
        # Calculate safe velocity
        safe_velocity = self.calculate_safe_velocity(fused_obstacles)
        
        # Publish results
        self.safe_vel_pub.publish(safe_velocity)
        
        status = Bool()
        status.data = self.avoidance_active
        self.avoidance_status_pub.publish(status)
    
    def fuse_obstacles(self, vision_valid: bool, lidar_valid: bool) -> list:
        """Fuse obstacles from vision and LiDAR"""
        fused = []
        
        # If both sensors valid, merge with weighting
        if vision_valid and lidar_valid:
            # Add all LiDAR obstacles (usually more reliable for distance)
            fused.extend(self.lidar_obstacles)
            
            # Add vision obstacles that aren't already covered by LiDAR
            for v_obs in self.vision_obstacles:
                if not self.is_duplicate_obstacle(v_obs, self.lidar_obstacles):
                    fused.append(v_obs)
        
        # If only one sensor valid, use what we have
        elif lidar_valid:
            fused = self.lidar_obstacles
            if not fused:
                self.get_logger().debug('Using LiDAR only - no obstacles detected')
        elif vision_valid:
            fused = self.vision_obstacles
            if not fused:
                self.get_logger().debug('Using vision only - no obstacles detected')
        
        return fused
    
    def is_duplicate_obstacle(self, obs1, obstacles_list, threshold=1.5):
        """Check if obstacle is duplicate within threshold distance"""
        for obs2 in obstacles_list:
            dist = math.sqrt(
                (obs1.position.x - obs2.position.x)**2 +
                (obs1.position.y - obs2.position.y)**2 +
                (obs1.position.z - obs2.position.z)**2
            )
            if dist < threshold:
                return True
        return False
    
    def calculate_safe_velocity(self, obstacles: list) -> Twist:
        """Calculate safe velocity based on fused obstacles"""
        if not obstacles:
            self.avoidance_active = False
            return self.requested_velocity
        
        # Find closest obstacle
        closest_dist = float('inf')
        closest_obs = None
        
        for obs in obstacles:
            # Calculate distance in UE4 coordinates
            dist = math.sqrt(
                obs.position.x**2 + 
                obs.position.y**2 + 
                obs.position.z**2
            )
            
            if dist < closest_dist:
                closest_dist = dist
                closest_obs = obs
        
        # Determine avoidance mode
        if closest_dist < self.critical_distance:
            mode = AvoidanceMode.EMERGENCY_STOP
            self.get_logger().warn(f'Emergency stop! Obstacle at {closest_dist:.1f}m')
        elif closest_dist < self.safety_distance:
            mode = self.determine_avoidance_mode(closest_obs, obstacles)
            self.get_logger().info(f'Avoiding obstacle at {closest_dist:.1f}m')
        else:
            mode = AvoidanceMode.SLOW_DOWN
        
        # Apply avoidance
        if mode == AvoidanceMode.NONE:
            self.avoidance_active = False
            return self.requested_velocity
        else:
            self.avoidance_active = True
            return self.apply_avoidance(mode, closest_obs, obstacles)
    
    def determine_avoidance_mode(self, closest_obs, all_obstacles) -> AvoidanceMode:
        """Determine best avoidance strategy"""
        # Check if we can go around laterally
        left_clear = self.is_direction_clear(all_obstacles, -1, 0)  # Left
        right_clear = self.is_direction_clear(all_obstacles, 1, 0)  # Right
        up_clear = self.is_direction_clear(all_obstacles, 0, 1)     # Up
        
        if left_clear or right_clear:
            return AvoidanceMode.LATERAL_AVOID
        elif up_clear:
            return AvoidanceMode.VERTICAL_AVOID
        else:
            return AvoidanceMode.EMERGENCY_STOP
    
    def is_direction_clear(self, obstacles: list, x_dir: float, z_dir: float) -> bool:
        """Check if a direction is clear of obstacles"""
        # Check 2 meters in the given direction
        check_distance = 2.0
        
        for obs in obstacles:
            # Project obstacle position
            if x_dir != 0:  # Lateral check
                if abs(obs.position.x - x_dir * check_distance) < 1.0:
                    return False
            if z_dir != 0:  # Vertical check
                if abs(obs.position.z - z_dir * check_distance) < 1.0:
                    return False
        
        return True
    
    def apply_avoidance(self, mode: AvoidanceMode, closest_obs, all_obstacles) -> Twist:
        """Apply the selected avoidance mode"""
        safe_vel = Twist()
        
        if mode == AvoidanceMode.EMERGENCY_STOP:
            # Full stop
            return safe_vel
        
        elif mode == AvoidanceMode.SLOW_DOWN:
            # Reduce speed by 60%
            safe_vel = self.requested_velocity
            safe_vel.linear.x *= 0.4
            safe_vel.linear.y *= 0.4
            
        elif mode == AvoidanceMode.LATERAL_AVOID:
            # Determine best lateral direction
            left_clear = self.is_direction_clear(all_obstacles, -1, 0)
            right_clear = self.is_direction_clear(all_obstacles, 1, 0)
            
            if left_clear and not right_clear:
                safe_vel.linear.x = self.avoidance_speed  # Go left
            elif right_clear and not left_clear:
                safe_vel.linear.x = -self.avoidance_speed  # Go right
            else:
                # Both clear, choose based on obstacle position
                if closest_obs.position.x > 0:
                    safe_vel.linear.x = self.avoidance_speed  # Go left
                else:
                    safe_vel.linear.x = -self.avoidance_speed  # Go right
            
            # Maintain reduced forward speed
            safe_vel.linear.y = self.requested_velocity.linear.y * 0.3
            
        elif mode == AvoidanceMode.VERTICAL_AVOID:
            # Go up and over
            safe_vel.linear.z = self.avoidance_speed
            safe_vel.linear.y = self.requested_velocity.linear.y * 0.5
        
        return safe_vel


def main(args=None):
    rclpy.init(args=args)
    node = FusionObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()