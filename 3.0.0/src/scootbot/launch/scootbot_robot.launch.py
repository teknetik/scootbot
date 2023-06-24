import os
import launch_ros
import launch
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart

import xacro


def generate_launch_description():

    package_name='scootbot'

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    
    # Process the URDF file
    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name)
    default_rviz_config_path = os.path.join(pkg_share, 'config/scootbot_ros2_control.rviz')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, use_ros2_control: 'true'}.items()
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
            }],
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'use_sim_time': use_sim_time
            }],
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{
            'use_sim_time': use_sim_time
            }],
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','controllers.yaml')

    controller_manger = Node (
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file],
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_ros2_control'))
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manger])

    diff_drive_spawner = Node (
        package="controller_manager",
        executable="spawner",
        arguments=["diff_ctl"],
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_ros2_control'))
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manger,
            on_start=[diff_drive_spawner],
        )
    )
    joint_broadcast_spawner = Node (
        package="controller_manager",
        executable="spawner",
        arguments=["joint_bcast"],
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_ros2_control'))
    )
    delayed_joint_broadcast_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manger,
            on_start=[joint_broadcast_spawner],
        )
    )
    # Launch!
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument('joy_config', default_value='ps3'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('scootbot'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control true'),
        DeclareLaunchArgument(
            name='rvizconfig', 
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'),

        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.4,
                'autorepeat_rate': 10.0,
            }]),
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', 
            remappings=[('/cmd_vel', '/diff_ctl/cmd_vel_unstamped')],
            parameters=[config_filepath],
            condition=launch.conditions.IfCondition(LaunchConfiguration('use_ros2_control'))
            ),
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', 
            parameters=[config_filepath],
            condition=launch.conditions.UnlessCondition(LaunchConfiguration('use_ros2_control'))
            ),
        #rsp,
        joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        delayed_controller_manager,
        #rviz_node,
        delayed_diff_drive_spawner,
        delayed_joint_broadcast_spawner
    ])
    
