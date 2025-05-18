from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('golf_cart')
    
    # Robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': open(os.path.join(pkg_share, 'urdf', 'summit_xls.urdf.xacro'), 'r').read()},
            {'use_sim_time': True}
        ]
    )
    
    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher
    ])