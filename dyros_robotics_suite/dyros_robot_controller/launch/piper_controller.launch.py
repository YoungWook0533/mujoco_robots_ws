from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_node = Node(
        package='mujoco_ros_sim',
        executable='mujoco_ros_sim',
        name='mujoco_sim_node',
        output='screen',
        parameters=[
            {'robot_name': 'agilex_piper'},
            {'controller_class': 'dyros_robot_controller.PiperController'},
        ],
    )

    urdf_path = os.path.join(
        get_package_share_directory('dyros_robot_controller'),
        'robot', 
        'piper.urdf')
    
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        sim_node,
        robot_state_publisher,
        joint_state_publisher
    ])