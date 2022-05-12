source ~/scootbot/install/setup.sh
sudo chmod 777 /dev/ttyS0
ros2 run sabertooth drive &
#ros2 run teleop_twist_joy teleop_node teleop_node &
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' &
