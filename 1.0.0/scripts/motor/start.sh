source ~/scootbot/install/setup.sh &
~/scootbot/scripts/motor/permissions.sh &
ros2 launch src/sabertooth/launch/sabertooth_launch_file.launch.py &
ros2 launch sllidar_ros2 sllidar_launch.py &
ros2 launch phidgets_spatial spatial-launch.py &
ros2 launch imu_complementary_filter complementary_filter.launch.py &
ros2 launch differential_drive pid.launch.py &