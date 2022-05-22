source ~/scootbot/install/setup.sh
sudo chmod 777 /dev/ttyS0
ros2 run sabertooth drive &
ros2 run differential_drive twist_to_motors &
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' &
ros2 run joy_cmd joy_cmd_node &
