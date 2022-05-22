# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from sqlalchemy import false
import rclpy
import RPi.GPIO as GPIO
import time
from sensor_msgs.msg import Joy


# Setup GPIO pins
# GPIO.setmode(GPIO.BCM)
# pinList = [26, 19, 13, 6]

# for i in pinList:
#     GPIO.setup(i, GPIO.OUT)

g_node = None

button_map=[]
lights={}
blades={}

lights['name'] = 'Lights'
lights['status']=False
lights['relay_pin']=13

blades['name'] = 'Blades'
blades['status']=False
blades['relay_pin']=19

button_map.append('None')
button_map.append(blades)
button_map.append('None')
button_map.append(lights)

def chatter_callback(msg):
    global g_node
    #print(msg)
    # get the index value (idb) of the button (b) in the topic msg (joy)
    for idb, b in enumerate(msg.buttons):
        if b == 1:
            try:
                print(button_map[int(idb)]['name'], button_map[int(idb)]['relay_pin'])
                print(type(button_map[int(idb)]['status']))
                if button_map[int(idb)]['status']:
                    print(button_map[int(idb)]['status'])
                    p = GPIO.output(button_map[int(idb)]['relay_pin'], GPIO.HIGH)
                    print(p)
                    button_map[int(idb)]['status']=False
                else:
                    print(button_map[int(idb)]['status'])
                    p = GPIO.output(button_map[int(idb)]['relay_pin'], GPIO.LOW)
                    print(p)
                    button_map[int(idb)]['status']=True
            except:
                pass



def relay(pin):
    print("pin = " + pin)
    #GPIO.setmode(GPIO.BCM)
    #GPIO.setup(i, GPIO.OUT)
    #GPIO.output(19, GPIO.LOW)

    return None


def main(args=None):
    global g_node
    rclpy.init(args=args)

    g_node = rclpy.create_node('joy_cmd_node')

    subscription = g_node.create_subscription(Joy, 'joy', chatter_callback, 10)
    subscription  # prevent unused variable warning

    while rclpy.ok():
        rclpy.spin_once(g_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    g_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
