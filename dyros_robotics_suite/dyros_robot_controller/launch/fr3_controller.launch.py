from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    sim_node = Node(
        package='mujoco_ros_sim',
        executable='mujoco_ros_sim',
        name='mujoco_sim_node',
        output='screen',
        parameters=[
            {'robot_name': 'franka_fr3_torque'},
            {'controller_class': 'dyros_robot_controller.FR3Controller'},
        ],
        # prefix=(
        #     'xterm -hold -e '
        #     'gdb -q '
        #     '-ex "set breakpoint pending on" '
        #     '-ex "set follow-fork-mode child" '
        #     '-ex "handle SIGUSR1 noprint nostop pass" '
        #     '-ex "run" --args python3'
        # ),
    )

    # urdf_path = os.path.join(
    #     get_package_share_directory('dyros_robot_controller'),
    #     'robot', 
    #     'fr3.urdf')
    urdf_path = os.path.join(
        get_package_share_directory('dyros_robot_controller'),
        'robot', 
        'fr3.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    rviz_config_file = os.path.join(
        get_package_share_directory("dyros_robot_controller"),
        "rviz", 
        "fr3_rviz.rviz"
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    gui_node = Node(
        package='dyros_robot_controller',
        executable='FR3ControllerQT',
        name='FR3ControllerQT',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        sim_node,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
        gui_node,
    ])