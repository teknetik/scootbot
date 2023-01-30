#!/bin/bash

source install/setup.sh
ros2 launch scootbot scootbot_sim.launch.py use_sim_time:=true & 
# Create Map
ros2 launch scootbot online_async_launch.py use_sim_time:=true &
# With Map
#ros2 launch scootbot slam_localization_launch.py use_sim_time:=true &
ros2 launch scootbot localization_launch.py use_sim_time:=true &
sleep 5
#ros2 run controller_manager spawner diff_ctl &
#ros2 run controller_manager spawner joint_bcast &
ros2 run nav2_util lifecycle_bringup map_server use_sim_time:=true &
ros2 launch scootbot humble_navigation_launch.py use_sim_time:=true &
