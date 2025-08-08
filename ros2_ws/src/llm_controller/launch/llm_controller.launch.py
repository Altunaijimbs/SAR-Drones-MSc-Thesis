from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    use_openai_arg = DeclareLaunchArgument(
        'use_openai',
        default_value='false',
        description='Use OpenAI API (true) or rule-based (false)'
    )
    
    velocity_topic_arg = DeclareLaunchArgument(
        'velocity_topic',
        default_value='/airsim_node/PX4/vel_cmd_body_frame',
        description='Velocity command topic'
    )
    
    # LLM Controller node
    llm_controller_node = Node(
        package='llm_controller',
        executable='llm_controller_node',
        name='llm_controller',
        parameters=[{
            'use_openai': LaunchConfiguration('use_openai'),
        }],
        remappings=[
            ('/drone/cmd_vel', LaunchConfiguration('velocity_topic'))
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_openai_arg,
        velocity_topic_arg,
        llm_controller_node
    ])
