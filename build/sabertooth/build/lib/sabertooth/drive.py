import rclpy
from rclpy.node import Node
from pysabertooth import Sabertooth
import time
from std_msgs.msg import Float32

saber = Sabertooth('/dev/ttyS0', baudrate=115200, address=128, timeout=0.1)

class CommandSubscriber(Node):

    def __init__(self):
        super().__init__('command_subscriber')
        self.lv= self.create_subscription(
            Float32,
            'lwheel_vtarget',
            self._velocity_received_callback_lw,
            10)
        self.rv= self.create_subscription(
            Float32,
            'rwheel_vtarget',
            self._velocity_received_callback_rw,
            10)

        self.command = False

    def _velocity_received_callback_lw(self, message):
       """Handle new velocity command message."""
       lv = message.data*10
       if message.data != 0.0:
           self.get_logger().info('lv= ' + str(lv))
       saber.drive(2, lv)

    def _velocity_received_callback_rw(self, message):
       """Handle new velocity command message."""
       rv = message.data*10
       if message.data != 0.0:
           self.get_logger().info('rv= ' + str(rv))
       saber.drive(1, rv)
        

def main(args=None):
    rclpy.init(args=args)
    command_subscriber = CommandSubscriber()
    rclpy.spin(command_subscriber)
    command_subscriber.destroy_node()
    rclpy.shutdown()
    saber.stop()

if __name__ == '__main__':
    main()
