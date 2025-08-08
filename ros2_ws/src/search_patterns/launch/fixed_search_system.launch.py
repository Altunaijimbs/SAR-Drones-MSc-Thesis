from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Parameters
    use_obstacle_avoidance = DeclareLaunchArgument(
        'use_obstacle_avoidance',
        default_value='true',
        description='Enable obstacle avoidance'
    )
    
    # Velocity Coordinator - MUST launch first
    velocity_coordinator_node = Node(
        package='search_patterns',
        executable='velocity_coordinator',
        name='velocity_coordinator',
        parameters=[{
            'command_timeout': 0.5,
            'publish_rate': 20.0,
        }],
        output='screen'
    )
    
    # Fixed Grid Search with proper QoS
    fixed_grid_search_node = Node(
        package='search_patterns',
        executable='fixed_grid_search',
        name='fixed_grid_search_node',
        parameters=[{
            'grid_spacing': 10.0,
            'search_speed': 2.0,
            'altitude': 20.0,
            'overlap_percentage': 20.0,
        }],
        output='screen'
    )
    
    # Obstacle avoidance (vision-based)
    obstacle_avoidance_node = Node(
        package='search_patterns',
        executable='obstacle_avoidance',
        name='obstacle_avoidance_node',
        parameters=[{
            'safety_distance': 3.0,
            'critical_distance': 1.5,
            'max_speed': 3.0,
            'avoidance_speed': 1.0,
        }],
        condition=LaunchConfiguration('use_obstacle_avoidance'),
        output='screen'
    )
    
    # Launch nodes with proper timing
    return LaunchDescription([
        use_obstacle_avoidance,
        
        # Launch velocity coordinator first
        velocity_coordinator_node,
        
        # Wait a bit then launch search
        TimerAction(
            period=1.0,
            actions=[fixed_grid_search_node]
        ),
        
        # Launch obstacle avoidance if enabled
        TimerAction(
            period=1.5,
            actions=[obstacle_avoidance_node]
        ),
    ])