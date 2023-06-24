#!/bin/bash

source install/setup.sh
rm -rf ~/.ros/log/*
# SIM
#ros2 launch scootbot scootbot_sim.launch.py use_ros2_control:=true use_sim_time:=true & 
# REAL
echo "publishing caster_frame_l_link"
ros2 run tf2_ros static_transform_publisher -0.22 0.165 0.025 0 0 0 1 base_link caster_frame_l_link > /dev/null 2>&1 &
echo "publishing caster_frame_r_link"
ros2 run tf2_ros static_transform_publisher -0.22 -0.165 0.025 0 0 0 1 base_link caster_frame_r_link > /dev/null 2>&1  &
echo "publishing casterwhl_l_link"
ros2 run tf2_ros static_transform_publisher -0.25 0.165 -0.08 0 0 0 1 base_link casterwhl_l_link > /dev/null 2>&1  &
echo "publishing casterwhl_r_link"
ros2 run tf2_ros static_transform_publisher -0.25 -0.165 -0.08 0 0 0 1 base_link casterwhl_r_link > /dev/null 2>&1  &

echo "Launching RSP"
ros2 launch scootbot rsp.launch.py use_sim_time:=false > log/robot_state_publisher.log 2>&1 &
sleep 5 

#Lidar
echo "Launching Lidar"
ros2 launch rplidar_ros rplidar.launch.py use_sim_time:=false > log/lidar.log 2>&1 &
sleep 5

echo "Launching Scootbot"
ros2 launch scootbot scootbot_robot.launch.py use_ros2_control:=true use_sim_time:=false > log/launch.log 2>&1 &
sleep 5

# Create Map
echo "Launching Online Async SLAM"
ros2 launch scootbot online_async_launch.py use_sim_time:=false > log/slam.log 2>&1 &


# With Map
#ros2 launch scootbot slam_localization_launch.py use_sim_time:=true &
sleep 10

echo "Launching localization"
ros2 launch scootbot localization_launch.py use_sim_time:=false > log/localization.log 2>&1 &
sleep 5

echo "Launching Nav2"
ros2 launch scootbot humble_navigation_launch.py use_sim_time:=false > log/nav2.log 2>&1 &
sleep 5

#ros2 launch phidgets_spatial spatial-launch.py &