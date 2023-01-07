from platform import node
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Int64, Float32








class pid(Node):
    
    def __init__(self):
        super().__init__('pid_controller')

        # Topics
        self.pub_motor = self.create_publisher(Float32, 'motor_cmd', 10)
        self.pub_vel = self.create_publisher(Float32, 'wheel_vel', 10)
        self.create_subscription(Int64, 'wheel', self.wheel_callback, 10) # Encoder
        self.create_subscription(Float32, 'wheel_vtarget', self.target_callback, 10) # Target Velocity

        # Vars
        self.then = self.get_clock().now()

    def wheel_callback(self, msg):
        enc = msg.data
        # self.get_logger().info("Raw Encoder data: " + str(enc))

    def target_callback(self, msg):
        enc = Float32()
        enc.data = msg.data
        self.get_logger().info("V Target " + str(enc.data))
        self.pub_motor.publish(enc)


    def calc_velocity(self):
            self.dt_duration = self.get_clock().now() - self.then
            self.dt = self.dt_duration.to_sec()

def main(args=None):
    rclpy.init(args=args)
    pid_controller = pid()

    rclpy.spin(pid_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
