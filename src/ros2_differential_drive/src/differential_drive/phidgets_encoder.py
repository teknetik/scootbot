from email.header import Header
from platform import node
import rclpy
from rclpy.node import Node
from Phidget22.Phidget import *
from Phidget22.Devices.Encoder import *
import time
from std_msgs.msg import Int64, Header








class PhidgetEncoder(Node):

    def __init__(self):
        super().__init__('phidget_encoder')

        self.publisher_r = self.create_publisher(Int64, 'lwheelenc', 10)
        self.publisher_l = self.create_publisher(Int64, 'rwheelenc', 10)

        self.encoder0 = Encoder()
        self.encoder0.setChannel(0)

        self.encoder1 = Encoder()
        self.encoder1.setChannel(1)

        self.encoder0.openWaitForAttachment(5000)
        self.encoder1.openWaitForAttachment(5000)
        

        self.rate_hz = self.declare_parameter("rate_hz", 4).value 
        self.create_timer(1.0/self.rate_hz, self.lwheel_callback)
        self.create_timer(1.0/self.rate_hz, self.rwheel_callback)


    def lwheel_callback(self):
        self.position = self.encoder0.getPosition()
        msg = Int64()
        msg.data = int(self.position)
        self.publisher_r.publish(msg)
        #self.get_logger().info('Publishing rwheel: "%s"' % msg.data)

    def rwheel_callback(self):
        self.position = self.encoder1.getPosition()
        msg = Int64()
        msg.data = int(self.position)
        self.publisher_l.publish(msg)
        #self.get_logger().info('Publishing lwheel: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    
    minimal_publisher = PhidgetEncoder()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    PhidgetEncoder.encoder0.close()
    PhidgetEncoder.encoder1.close()

if __name__ == '__main__':
    main()
