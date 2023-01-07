import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')


    return LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_config', default_value='xbox'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),

        Node(
            package='sabertooth',
            executable='drive',
            output='screen',
            name='motor_driver'),
        Node(
            package='differential_drive',
            executable='twist_to_motors',
            output='screen',
            name='twist_to_motors',
            parameters=[{
                'base_width': 0.5
            }]),
        Node(
            package='differential_drive',
            executable='diff_tf',
            output='screen',
            name='diff_tf'),
        Node(
            package='differential_drive',
            executable='phidgets_encoder',
            output='screen',
            name='wheel_encoders')
        # Node(
        #     package='differential_drive',
        #     executable='pid_velocity',
        #     output='screen',
        #     name='lpid_velocity',
        #     remappings=[
        #     ("wheel", "lwheelenc"),
        #     ("motor_cmd", "lmotor_cmd"),
        #     ("wheel_vtarget", "lwheel_vtarget"),
        #     ("wheel_vel", "lwheel_vel")
        #     ],
        #     parameters=[{
        #         'Kp': 1,
        #         'Ki': 0,
        #         'Kd': 0,
        #         'out_min': -100,
        #         'out_max': 100,
        #         'rate': 10,
        #         'timeout_ticks': 400,
        #         'rolling_pts': 5
        #     }],
        #     arguments=['--ros-args', '--log-level', 'debug']
        #    ),
        # Node(
        #     package='differential_drive',
        #     executable='pid',
        #     output='screen',
        #     name='rpid_velocity',
        #     remappings=[
        #     ("wheel", "rwheelenc"),
        #     ("motor_cmd", "rmotor_cmd"),
        #     ("wheel_vtarget", "rwheel_vtarget"),
        #     ("wheel_vel", "rwheel_vel")
        #     ],
        #     parameters=[{
        #         'Kp': 1,
        #         'Ki': 0,
        #         'Kd': 0,
        #         'out_min': -100,
        #         'out_max': 100,
        #         'rate': 30,
        #         'timeout_ticks': 400,
        #         'rolling_pts': 5
        #     }])

    ])

