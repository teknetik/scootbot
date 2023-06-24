echo "publishing caster_frame_l_link"
ros2 run tf2_ros static_transform_publisher -0.22 0.165 0.025 0 0 0 1 base_link caster_frame_l_link &
echo "publishing caster_frame_r_link"
ros2 run tf2_ros static_transform_publisher -0.22 -0.165 0.025 0 0 0 1 base_link caster_frame_r_link &
echo "publishing casterwhl_l_link"
ros2 run tf2_ros static_transform_publisher -0.25 0.165 -0.08 0 0 0 1 base_link casterwhl_l_link &
echo "publishing casterwhl_r_link"
ros2 run tf2_ros static_transform_publisher -0.25 -0.165 -0.08 0 0 0 1 base_link casterwhl_r_link &
ros2 launch scootbot scootbot_robot.launch.py use_ros2_control:=true use_sim_time:=false | grep ArduinoComms
