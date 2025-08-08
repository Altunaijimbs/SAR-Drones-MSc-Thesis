from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sar_web_platform',
            executable='web_server',
            name='sar_web_platform',
            output='screen',
            parameters=[{
                'host': '0.0.0.0',
                'port': 5000
            }]
        )
    ])