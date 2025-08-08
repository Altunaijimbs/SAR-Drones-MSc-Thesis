from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    safety_distance_arg = DeclareLaunchArgument(
        'safety_distance',
        default_value='3.0',
        description='Distance to start avoidance maneuvers'
    )
    
    critical_distance_arg = DeclareLaunchArgument(
        'critical_distance',
        default_value='1.5',
        description='Distance for emergency stop'
    )
    
    # Grid search node
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
    
    # Search pattern manager
    pattern_manager_node = Node(
        package='search_patterns',
        executable='search_pattern_manager',
        name='search_pattern_manager',
        output='screen'
    )
    
    # Obstacle avoidance node
    obstacle_avoidance_node = Node(
        package='search_patterns',
        executable='obstacle_avoidance',
        name='obstacle_avoidance_node',
        parameters=[{
            'safety_distance': LaunchConfiguration('safety_distance'),
            'critical_distance': LaunchConfiguration('critical_distance'),
            'max_speed': 3.0,
            'avoidance_speed': 1.0,
            'vertical_clearance': 2.0,
        }],
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
        safety_distance_arg,
        critical_distance_arg,
        grid_search_node,
        pattern_manager_node,
        obstacle_avoidance_node,
        velocity_mux_node,
    ])