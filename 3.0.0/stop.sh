#!/bin/bash

source install/setup.sh

# Kill the scootbot_sim_nodes
# killall -9 teleop_node
# killall -9 joy_node
# killall -9 robot_state_publisher
# killall -9 joint_state_publisher_gui
# killall -9 rviz2
killall -9 gzserver
killall -9 gzclient



# # Kill the async_slam_toolbox_node
# killall -9 async_slam_toolbox_node



# # Kill localization nodes
# killall -9 map_server
# killall -9 amcl
# killall -9 lifecycle_manager

kill -9 $(ps aux | grep 'ros/humble' | grep -v fastdds | grep -v fast-discovery-server | awk '{print $2}')
kill -9 $(ps aux | grep 'rplidar_ros' | awk '{print $2}')
ros2 daemon stop
sleep 2
ros2 daemon start