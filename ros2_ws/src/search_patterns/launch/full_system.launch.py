from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Parameters
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Use LiDAR for obstacle detection'
    )
    
    use_fusion_arg = DeclareLaunchArgument(
        'use_fusion',
        default_value='true',
        description='Use sensor fusion for obstacle avoidance'
    )
    
    # Search pattern nodes
    grid_search_node = Node(
        package='search_patterns',
        executable='grid_search',
        name='grid_search_node',
        parameters=[{
            'grid_spacing': 10.0,
            'search_speed': 2.0,
            'altitude': 20.0,
            'overlap_percentage': 20.0,
        }],
        output='screen'
    )
    
    pattern_manager_node = Node(
        package='search_patterns',
        executable='search_pattern_manager',
        name='search_pattern_manager',
        output='screen'
    )
    
    # LiDAR obstacle detection
    lidar_obstacle_node = Node(
        package='search_patterns',
        executable='lidar_obstacle_avoidance',
        name='lidar_obstacle_avoidance',
        parameters=[{
            'min_distance': 2.0,
            'sector_angle': 15.0,
            'max_range': 20.0,
            'height_threshold': 0.5,
        }],
        condition=LaunchConfiguration('use_lidar'),
        output='screen'
    )
    
    # Obstacle avoidance (choose fusion or vision-only)
    fusion_avoidance_node = Node(
        package='search_patterns',
        executable='fusion_obstacle_avoidance',
        name='fusion_obstacle_avoidance',
        parameters=[{
            'safety_distance': 3.0,
            'critical_distance': 1.5,
            'lidar_weight': 0.6,
            'avoidance_speed': 1.0,
        }],
        condition=LaunchConfiguration('use_fusion'),
        output='screen'
    )
    
    vision_only_avoidance_node = Node(
        package='search_patterns',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[{
            'safety_distance': 3.0,
            'critical_distance': 1.5,
            'max_speed': 3.0,
            'avoidance_speed': 1.0,
        }],
        condition='not ' + LaunchConfiguration('use_fusion'),
        output='screen'
    )
    
    # Velocity multiplexer
    velocity_mux_node = Node(
        package='search_patterns',
        executable='velocity_multiplexer',
        name='velocity_multiplexer',
        parameters=[{
            'timeout': 0.5,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        use_lidar_arg,
        use_fusion_arg,
        grid_search_node,
        pattern_manager_node,
        lidar_obstacle_node,
        fusion_avoidance_node,
        vision_only_avoidance_node,
        velocity_mux_node,
    ])