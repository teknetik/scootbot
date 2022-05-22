import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim or any other robot

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    print(data)

# Intializes everything
def start():
# publishing to "turtle1/cmd_vel" to control turtle1
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('joy_cmd')
    rospy.spin()

if __name__ == '__main__':
    start()