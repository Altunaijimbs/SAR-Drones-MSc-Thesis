from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    grid_spacing_arg = DeclareLaunchArgument(
        'grid_spacing',
        default_value='10.0',
        description='Spacing between grid lines in meters'
    )
    
    search_speed_arg = DeclareLaunchArgument(
        'search_speed',
        default_value='2.0',
        description='Search speed in m/s'
    )
    
    altitude_arg = DeclareLaunchArgument(
        'altitude',
        default_value='20.0',
        description='Search altitude in meters'
    )
    
    overlap_arg = DeclareLaunchArgument(
        'overlap_percentage',
        default_value='20.0',
        description='Overlap percentage between search passes'
    )
    
    # Grid search node
    grid_search_node = Node(
        package='search_patterns',
        executable='grid_search',
        name='grid_search_node',
        parameters=[{
            'grid_spacing': LaunchConfiguration('grid_spacing'),
            'search_speed': LaunchConfiguration('search_speed'),
            'altitude': LaunchConfiguration('altitude'),
            'overlap_percentage': LaunchConfiguration('overlap_percentage'),
        }],
        output='screen'
    )
    
    # Search pattern manager node
    pattern_manager_node = Node(
        package='search_patterns',
        executable='search_pattern_manager',
        name='search_pattern_manager',
        output='screen'
    )
    
    return LaunchDescription([
        grid_spacing_arg,
        search_speed_arg,
        altitude_arg,
        overlap_arg,
        grid_search_node,
        pattern_manager_node,
    ])