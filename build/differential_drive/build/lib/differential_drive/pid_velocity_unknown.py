#!/usr/bin/env python

# Copyright (C) 2012 Jon Stephan.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int64



class pid_velocity(Node):
    """

      pid_velocity - takes messages on wheel_vtarget
                     target velocities for the wheels 
                     and monitors wheel for feedback

      Notes:  

      28815     clicks p/m
      360       encoder clicks per revolution
      3000      max rpm on the encoder

      ### = code comments
      # = commented out code 

    ros2 topic pub -r 10 /lwheel_vtraget std_msgs/Float32 20 -r 10
    ros2 topic pub /lwheel_vtraget std_msgs/msg/Float32 '{data: 1.0}'
    """


    def __init__(self):
        super().__init__("pid_velocity")
        self.nodename = "pid_velocity"
        self.get_logger().info(f"{self.nodename} started")

        self.target = 0.0
        self.vel_shape = 0.0

        self.enc = {"time" : self.get_clock().now(), "clicks" : 0}
        self.enc_last =  {"time" : self.get_clock().now(), "clicks" : 0}
        self.enc_delta = 0
        self.base = 0.55
        self.distance = 0
        self.mps = 0 
        self.miss = 0

        ### Encoder clicks per metre
        self.cpm = 57631

        self.time_now = self.get_clock().now()
        self.time_then = self.time_now

        self.create_subscription(Int64, 'lwheelenc', self.wheel_callback, 10)
        self.create_subscription(Float32, 'lwheel_vtarget', self.target_callback, 10)
        self.get_logger().debug("create subs")
        self.pub_motor = self.create_publisher(Float32, 'lmotor_cmd', 10)
        self.pub_motor = self.create_publisher(Float32, 'rmotor_cmd', 10)
        self.pub_vel = self.create_publisher(Float32, 'lwheel_vel', 10)

        self.rate_hz = self.declare_parameter("rate_hz", 10).value
        
        self.create_timer(1.0/self.rate_hz, self.get_vel)
        #self.create_timer(1.0/self.rate_hz, self.vel_shaper)

    
    
    def get_vel(self):

        
        


        

        # self.get_logger().info("-------SHAPING--------") 
        # self.get_logger().info("MPS: " + str(mps))
        # self.get_logger().info("Target: " + str(self.target))
        # self.get_logger().info("Shape: " + str(self.vel_shape) + "\n --- \n\n")

        
        lwheel_cmd = Float32()    
        lwheel_cmd.data = self.target
        self.pub_motor.publish(lwheel_cmd)
        
        rwheel_cmd = Float32()    
        rwheel_cmd.data = self.target
        self.pub_motor.publish(rwheel_cmd)


    ### Save last encoder value
        self.enc_last = self.enc
    ### Save last time stamp
        self.time_then = self.time_now
    
    
    
    ### Callbacks

    def wheel_callback(self, msg):


        ### Get current time minus time of previous mesage (at 10Mhz should 0.1s) to provide the time delta between econder messages
        self.time_now = self.get_clock().now()
        self.enc = {"time" : self.get_clock().now(), "clicks" : msg.data}
        self.get_logger().info("lwheelenc: %s" % (self.enc))

        self.dt_duration = self.enc['time'] - self.enc_last['time']
        #self.get_logger().info("time delta " + str(round(self.dt_duration.nanoseconds / 1e+9, 2)))
        self.dt = round(self.dt_duration.nanoseconds / 1e+9, 2)

        ### Get delta between current encoder click and previous encoder clicks
        self.enc_delta = self.enc['clicks'] - self.enc_last['clicks']

        if self.enc_delta == self.enc_delta:
            self.miss += 1

        ### Distance travelled in metres is enc_delta / CPM
        self.distance = self.enc_delta / self.cpm
        try:
            mps = round(1/self.dt * self.distance, 10)
        except ZeroDivisionError as e:
            self.get_logger().info("DIST EXCEPT") 
            mps=0
            pass            
        
        self.get_logger().info("-------DELTA--------") 
        self.get_logger().info("enc_delta: " + str(self.enc_delta))
        self.get_logger().info("delta_time: " + str(self.dt)) 
        self.get_logger().info("mps: " + str(mps) + "m/s\n") 

    def target_callback(self, msg):
        self.target = msg.data



def main(args=None):
    rclpy.init(args=args)
    try:
        l_pid_velocity = pid_velocity()
        rclpy.spin(l_pid_velocity)
    except rclpy.exceptions.ROSInterruptException:
        pass

    pid_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()