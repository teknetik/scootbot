import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='differential_drive',
            executable='pid',
            output='screen',
            name='rpid_velocity',
            remappings=[
            ("wheel", "rwheelenc"),
            ("motor_cmd", "rmotor_cmd"),
            ("wheel_vtarget", "rwheel_vtarget"),
            ("wheel_vel", "rwheel_vel")
            ],
            parameters=[{
                'Kp': 1,
                'Ki': 0,
                'Kd': 0,
                'out_min': -100,
                'out_max': 100,
                'rate': 30,
                'timeout_ticks': 400,
                'rolling_pts': 5
            }])

    ])

