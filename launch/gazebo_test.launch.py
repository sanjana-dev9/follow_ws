from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the Gazebo launch directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Simple URDF content for a box
    simple_urdf = """<?xml version="1.0"?>
    <robot name="simple_box">
        <link name="base_link">
            <visual>
                <geometry>
                    <box size="0.5 0.5 0.5"/>
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <box size="0.5 0.5 0.5"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="1.0"/>
                <inertia ixx="0.083" ixy="0.0" ixz="0.0" iyy="0.083" iyz="0.0" izz="0.083"/>
            </inertial>
        </link>
    </robot>
    """
    
    # Launch Gazebo using the provided launch file
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
        parameters=[{'robot_description': simple_urdf, 'use_sim_time': True}]
    )
    
    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_box',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen',
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])