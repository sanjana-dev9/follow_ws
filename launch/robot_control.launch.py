from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_golf_cart = get_package_share_directory('golf_cart')
    
    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    prefix = LaunchConfiguration('prefix')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_prefix_cmd = DeclareLaunchArgument(
        'prefix',
        default_value='summit_xl_',
        description='Prefix for TF frames'
    )
    
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(pkg_golf_cart),'launch','robot_state_publisher.launch.py'
            )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Launch gazebo with the proper launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'summit_xl',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen',
    )
    
    # Controller spawner
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'robotnik_base_control'],
        output='screen',
    )

    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_golf_cart, 'rviz', 'robot_state.rviz')],
    )
    
    # Add twist_mux nodes if needed
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(pkg_golf_cart, 'config', 'twist_mux.yaml')],
        remappings=[
            ('cmd_vel_out', 'robotnik_base_control/cmd_vel')
        ],
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_prefix_cmd,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        controller_spawner,
        rviz_node,
        twist_mux
    ])