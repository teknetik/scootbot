from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sabertooth',
            executable='drive',
            output='screen',
            name='sabertooth'),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            output='screen',
            name='teleop_twist'),
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            name='joy',
            parameters=[
                {"dev": "/dev/input/event0"}
        ])
    ])
