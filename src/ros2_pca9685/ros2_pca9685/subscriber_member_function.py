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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

print("Initializing IO System - import")
import time
import adafruit_pca9685
from adafruit_servokit import ServoKit
import serial
import board
import busio
#import signal

kit = ServoKit(channels=16, address=0x40)
print("Initializing IO System - kit")

i2c = busio.I2C(board.SCL, board.SDA)
print("Initializing IO System - board")
pca = adafruit_pca9685.PCA9685(i2c)
print("Initializing IO System - pca")

#kit.set_pwm_freq(50)
#pca.setPWMFreq(50)
print("Initializing IO System - freq")
pca.frequency = 50


maxr=135
minl=30
# Forward
forward_max_thr =  125 # checkin 125 
forward_min_thr =  65 # checking 65
# Backwards
backwards_max_thr = 125
backwards_min_thr = 75 #65, 75 for parallel parking

thr_init = 90 
str_init = 87 # used to be 100, checkin 87 & paralell parking

pin_STR = 15
pin_THR = 14

print("Initializing Propulsion System")
kit.servo[pin_THR].angle = thr_init
time.sleep(1)

print("Initializing Steering System")
kit.servo[pin_STR].angle = str_init

bs1 = 180 - thr_init
kit.servo[2].angle = bs1

print("Listener Node Started...")


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg: Twist):
        throttle = -msg.linear.x
        steering = msg.angular.z
        # self.get_logger().info('Throttle: "%s"' % throttle)
        # self.get_logger().info('Steering: "%s"' % steering)
        
        old_str_value = float(steering)
        old_thr_value = float(throttle)

        if old_str_value < 0:
            old_str_value = -old_str_value
            new_str_value = int(str_init + (old_str_value * ((maxr-str_init)/3)))
        else:
            new_str_value = int(str_init - (old_str_value * ((str_init-minl)/3)))

        if new_str_value > maxr:
            new_str_value = maxr
        if new_str_value < minl:
            new_str_value = minl           

        if old_thr_value < 0: # forward
            max_thr = forward_max_thr
            min_thr = forward_min_thr
        else: # backward
            max_thr = backwards_max_thr
            min_thr = backwards_min_thr
            
        new_thr_range = max_thr-min_thr
        new_thr_value = int(((old_thr_value) * (new_thr_range)+90))

        if new_thr_value > max_thr:
            new_thr_value = max_thr 

        if new_thr_value < min_thr:
            new_thr_value = min_thr

        move_robot(new_thr_value, new_str_value)
        
def move_robot(thr_num: float, str_num: float):
    # print("Moving Robot:  Throttle=" + str(thr_num) + " ,  Steering=" + str(str_num))
    kit.servo[pin_THR].angle = thr_num

    kit.servo[pin_STR].angle = str_num
    if str_num > str_init:
        bs1 = str_init - (str_num - str_init)
    else:
        bs1 = str_init + (str_init - str_num)

    kit.servo[2].angle = bs1        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)

        minimal_subscriber.destroy_node()
        rclpy.shutdown()
    except:
        move_robot(thr_init, str_init)
        
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
