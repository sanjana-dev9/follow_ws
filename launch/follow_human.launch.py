from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription

def generate_launch_description():
    # Declare launch arguments for person detection
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
    
    # Declare launch arguments for navigation
    person_radius_arg = DeclareLaunchArgument(
        'person_radius',
        default_value='2.5',
        description='Distance to maintain from person in meters'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to map file (leave empty for SLAM)'
    )
    
    # Nav2 parameters file argument
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('golf_cart'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Path to Nav2 parameters file'
    )
    
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Use SLAM for mapping'
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map_file'),
            'use_sim_time': 'true',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_slam'))
    )
    
    # SLAM Toolbox launch (if using SLAM)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam'))
    )
    
    # Nav2 without map (for SLAM)
    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': LaunchConfiguration('nav2_params_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam'))
    )
    
    # Person detection node
    person_detector_node = Node(
        package='golf_cart',
        executable='detect_human',
        name='detect_human',
        output='screen',
        parameters=[{
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'display_window': True,
            'publish_annotated': True,
            'confidence_threshold': 0.5,
            'iou_threshold': 0.4,
            'model_path': 'yolov8n.pt',
        }]
    )
    
    # Person following navigator node
    follow_human_node = Node(
        package='golf_cart',
        executable='follow_human',
        name='follow_human',
        output='screen',
        parameters=[{
            'person_radius': LaunchConfiguration('person_radius'),
            'base_frame': 'summit_xl_base_footprint',
            'map_frame': 'map',
            'camera_frame': 'summit_xl_front_rgbd_camera_link',
            'navigation_timeout': 30.0,
            'min_person_distance': 0.5,
            'max_person_distance': 10.0,
        }]
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('golf_cart'),
            'config',
            'person_following.rviz'
        ])],
    )
    
    return LaunchDescription([
        rgb_topic_arg,
        pointcloud_topic_arg,
        person_radius_arg,
        map_file_arg,
        use_slam_arg,
        nav2_params_arg,
        slam_launch,
        nav2_launch,
        nav2_slam_launch,
        person_detector_node,
        # detect_human_node,
        rviz_node
    ])