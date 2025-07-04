from launch import LaunchDescription
from launch import actions, substitutions
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    config_file = os.path.join(
        get_package_share_directory("dyros_robot_controller"),
        "config",
        "teleop_twist_joy.yaml"
    )
    
    declare_args = [
        actions.DeclareLaunchArgument('joy_vel',
            default_value=TextSubstitution(text='cmd_vel')),
        actions.DeclareLaunchArgument('publish_stamped_twist',
            default_value=TextSubstitution(text='false')),
        actions.DeclareLaunchArgument('joy_dev',
            default_value=TextSubstitution(text='0')),
        actions.DeclareLaunchArgument('config_filepath',
            default_value=TextSubstitution(text=config_file)),
        actions.DeclareLaunchArgument('robot_name',
            default_value=TextSubstitution(text='dyros_pcv')),
    ]

    # LaunchConfigurations
    joy_vel                = LaunchConfiguration('joy_vel')
    publish_stamped_twist  = LaunchConfiguration('publish_stamped_twist')
    joy_dev                = LaunchConfiguration('joy_dev')
    config_filepath        = LaunchConfiguration('config_filepath')
    robot_name_cfg         = LaunchConfiguration('robot_name')
    
    sim_node = Node(
        package='mujoco_ros_sim',
        executable='mujoco_ros_sim',
        name='mujoco_sim_node',
        output='screen',
        parameters=[
            {'robot_name': robot_name_cfg},
            {'controller_class': 'dyros_robot_controller.PCVController'},
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

    # rviz_config_file = os.path.join(
    #     get_package_share_directory("dyros_robot_controller"),
    #     "launch", 
    #     "fr3_rviz.rviz"
    # )

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[
    #         robot_description,
    #         {'use_sim_time': True}
    #     ]
    # )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         moveit_config.joint_limits,
    #     ],
    # )

    gui_node = Node(
        package='dyros_robot_controller',
        executable='PCVControllerQT',
        name='PCVControllerQT',
        output='screen',
        emulate_tty=True,
    )
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }],
    )
    
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[
            config_filepath,
            {'publish_stamped_twist': publish_stamped_twist},
        ],
        remappings=[
            # ('cmd_vel', joy_vel),
            ('cmd_vel', '/pcv_controller/cmd_vel'),
        ],
    )


    return LaunchDescription(
        declare_args +[
        sim_node,
        # robot_state_publisher,
        joint_state_publisher,
        # rviz_node,
        gui_node,
        joy_node,
        teleop_twist_joy_node,
        ]
    )