from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    # MAVROS node
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='',
        parameters=[{
            'fcu_url': 'tcp://127.0.0.1:4560',
            'target_system_id': 1,
            'target_component_id': 1,
        }],
        output='screen'
    )
    
    # Continuous setpoint publisher
    setpoint_pub = Node(
        package='llm_controller',
        executable='keep_alive_node',
        name='keep_alive',
        output='screen'
    )
    
    # Wait then set mode (fixed syntax)
    set_mode_cmd = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/mavros/set_mode', 'mavros_msgs/srv/SetMode', '{"custom_mode": "OFFBOARD"}'],
                shell=True
            )
        ]
    )
    
    # Arm command
    arm_cmd = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/mavros/cmd/arming', 'mavros_msgs/srv/CommandBool', '{"value": true}'],
                shell=True
            )
        ]
    )
    
    return LaunchDescription([
        mavros,
        TimerAction(period=2.0, actions=[setpoint_pub]),
        set_mode_cmd,
        arm_cmd
    ])
