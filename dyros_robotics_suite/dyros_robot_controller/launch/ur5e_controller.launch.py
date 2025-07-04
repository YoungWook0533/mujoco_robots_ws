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
            {'robot_name': 'universal_robots_ur5e'},
            {'controller_class': 'dyros_robot_controller.UR5eController'},
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

    urdf_path = os.path.join(
        get_package_share_directory('dyros_robot_controller'),
        'robot', 
        'ur5e.urdf')
    moveit_config_path = get_package_share_directory('ur5e_moveit_config')
    moveit_config = (
        MoveItConfigsBuilder("ur5e")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(
            file_path=os.path.join(moveit_config_path, "config", "ur5e.srdf"))
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    rviz_config_file = os.path.join(
        get_package_share_directory("dyros_robot_controller"),
        "rviz", 
        "ur5e_rviz.rviz"
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': True}
        ]
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
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    gui_node = Node(
        package='dyros_robot_controller',
        executable='UR5eControllerQT',
        name='UR5eControllerQT',
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