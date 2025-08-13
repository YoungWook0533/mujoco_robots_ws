from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory('dyros_robot_controller')

    sim_node = Node(
    package='mujoco_ros_sim',
    executable='mujoco_ros_sim',
    name='mujoco_sim_node',
    output='screen',
    parameters=[
        {'robot_name': 'fr3_husky'},
        {'controller_class': 'dyros_robot_controller.HuskyFR3Controller'},
        # Use external MPPI bridge
        {'use_external_mppi': True},
    ],
)

    urdf_path = os.path.join(
        pkg_share,
        'robot', 'husky_fr3_vis.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    rviz_config_file = os.path.join(
        pkg_share,
        "rviz", 
        "husky_fr3_rviz.rviz"
    )

    # External MPPI node
    mppi_config_path = os.path.join(pkg_share, 'config', 'mppi_husky_fr3.yaml')
    mppi_node = Node(
        package='mppi_ros',
        executable='husky_fr3_mppi_node',
        name='husky_fr3_mppi',
        output='screen',
        parameters=[
            # Required by ControllerRos
            {'policy_update_rate': 20.0},
            {'reference_update_rate': 10.0},
            {'ros_publish_rate': 20.0},
            {'publish_ros': True},
            # Husky+FR3 specifics
            {'dt': 0.01},
            {'mppi_config_path': mppi_config_path},
            {'urdf_path': os.path.join(pkg_share, 'robot', 'husky_fr3.urdf')},
            {'goal_topic': 'husky_fr3_controller/target_pose'},
            {'observation_topic': 'husky_fr3_controller/mppi_observation'},
        ]
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
        executable='HuskyFR3ControllerQT',
        name='HuskyFR3ControllerQT',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        sim_node,
        mppi_node,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
        gui_node,
    ])