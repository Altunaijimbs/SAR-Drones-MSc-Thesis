from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/airsim_node/drone_1/front_center/Scene',
        description='Camera topic to subscribe to'
    )
    
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov8m.pt',
        description='YOLO model to use (yolov8n/s/m/l/x)'
    )
    
    # Nodes
    vision_to_text_node = Node(
        package='drone_vision_interpreter',
        executable='vision_to_text_node',
        name='vision_to_text',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'yolo_model': LaunchConfiguration('yolo_model'),
            'enable_visualization': True,
            'confidence_threshold': 0.4,
            'process_every_n_frames': 3
        }],
        output='screen'
    )
    
    object_detector_node = Node(
        package='drone_vision_interpreter',
        executable='object_detector_node',
        name='object_detector',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'yolo_model': 'yolov8s.pt',  # Use smaller model for obstacle detection
            'confidence_threshold': 0.5
        }],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        camera_topic_arg,
        yolo_model_arg,
        vision_to_text_node,
        object_detector_node
    ])
