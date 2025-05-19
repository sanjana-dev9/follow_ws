from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/orbbec_astra/front_rgbd_camerargb/image_raw_color',
        description='RGB image topic'
    )
    
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/orbbec_astra/front_rgbd_camerapoint_cloud/cloud_registered',
        description='Point cloud topic'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Path to YOLO model file'
    )
    
    # Human Detector Node with Point Cloud
    human_detector_node = Node(
        package='golf_cart',
        executable='detect_human',
        name='detect_human',
        output='screen',
        parameters=[{
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'model_path': LaunchConfiguration('model_path'),
        }]
    )
    
    return LaunchDescription([
        rgb_topic_arg,
        pointcloud_topic_arg,
        model_path_arg,
        human_detector_node
    ])