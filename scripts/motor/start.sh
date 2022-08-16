source ~/scootbot/install/setup.sh
sudo chmod 777 /dev/ttyS0
sudo chmod 777 /dev/input/event0
sudo chmod 777 /dev/gpiomem
ros2 run sabertooth drive &
ros2 run differential_drive twist_to_motors &
ros2 run differential_drive diff_tf &

ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' &
ros2 run joy_cmd joy_cmd_node &
ros2 launch phidgets_spatial spatial-launch.py &
