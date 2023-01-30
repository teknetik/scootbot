#!/bin/bash

source install/setup.sh
# SIM
#ros2 launch scootbot scootbot_sim.launch.py use_ros2_control:=true use_sim_time:=true & 
# REAL
ros2 launch scootbot scootbot_robot.launch.py use_ros2_control:=true use_sim_time:=false & 


# Create Map
ros2 launch scootbot online_async_launch.py use_sim_time:=true &
# With Map
#ros2 launch scootbot slam_localization_launch.py use_sim_time:=true &


ros2 launch scootbot localization_launch.py use_sim_time:=true &
sleep 5

ros2 launch scootbot humble_navigation_launch.py use_sim_time:=true &
