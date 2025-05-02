import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg._laser_scan import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from enum import Enum, auto

import numpy as np
import time

from util.lidar import Lidar, LidarAngles

# distance in meters
SLOW_SPEED = 0.15
SMALL_WALL_LEN = 0.4572  # 18 inches
BIG_WALL_LEN = 0.762     # 30 inches
MIN_GAP = 0.15
TURN_RATE = 10.0

SHIMMY_RIGHT_TIME = 0.1
SHIMMY_LEFT_TIME = 0.1
SHIMMY_FORWARD_TIME = 0.1

SHIMMY_DONE_DIST = 0.1

class ParkingState(Enum):
    FIND_START     = auto()
    FIND_END       = auto()

    # Shimmy subroutine
    SHIMMY_RIGHT   = auto() # do for x secs
    SHIMMY_LEFT    = auto() # do for x secs
    SHIMMY_FORWARD = auto() # do for x secs

    STRAIGHTEN_OUT = auto() # try to adjust angle of self to straighten out

    DONE           = auto()


class SpotDetectionAndParking(Node):
    def __init__(self):
        super().__init__('spot_detection_parking_node')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.state: ParkingState = ParkingState.FIND_START

        self.lidar = Lidar()
        self.front_dist = math.nan
        self.back_dist  = math.nan
        self.left_dist  = math.nan
        self.right_dist = math.nan

        self.prev_front_dist = self.front_dist
        self.prev_back_dist  = self.back_dist
        self.prev_left_dist  = self.left_dist
        self.prev_right_dist = self.right_dist

        self.delta_front_dist = self.front_dist - self.prev_front_dist
        self.delta_back_dist  = self.back_dist  - self.prev_back_dist 
        self.delta_left_dist  = self.left_dist  - self.prev_left_dist 
        self.delta_right_dist = self.right_dist - self.prev_right_dist

        self.start_time = time.time()

    def scan_callback(self, msg: LaserScan):
        self.lidar.update(msg)

        # Get previous results
        self.prev_front_dist = self.front_dist  
        self.prev_back_dist  = self.back_dist   
        self.prev_left_dist  = self.left_dist   
        self.prev_right_dist = self.right_dist  

        # Get current results
        self.front_dist = self.lidar.get_dist(LidarAngles.FRONT_E.value)
        self.back_dist  = self.lidar.get_dist(LidarAngles.BACK_E.value)
        self.left_dist  = self.lidar.get_dist(LidarAngles.LEFT_E.value)
        self.right_dist = self.lidar.get_dist(LidarAngles.RIGHT_E.value)

        # Get delta results
        self.delta_front_dist = self.front_dist - self.prev_front_dist
        self.delta_back_dist  = self.back_dist  - self.prev_back_dist 
        self.delta_left_dist  = self.left_dist  - self.prev_left_dist 
        self.delta_right_dist = self.right_dist - self.prev_right_dist

        # Manage state
        self.state_update()

    def state_update(self):
        self.get_logger().info(f'[state_update] {self.state}')

        if self.state == ParkingState.FIND_START:
            self.find_start()
        elif self.state == ParkingState.FIND_END:
            self.find_end()
        elif self.state == ParkingState.SHIMMY_RIGHT:
            self.shimmy_right()
        elif self.state == ParkingState.SHIMMY_LEFT:
            self.shimmy_left()
        elif self.state == ParkingState.SHIMMY_FORWARD:
            self.shimmy_forward()
        elif self.state == ParkingState.STRAIGHTEN_OUT:
            self.straighten_out()
        elif self.state == ParkingState.DONE:
            self.done()

    def find_start(self):
        if abs(self.delta_right_dist) >= SMALL_WALL_LEN / 2:
            self.state = ParkingState.FIND_END
            self.state_update()
            return
        self.move(SLOW_SPEED, 0.0)
    def find_end(self):
        if abs(self.delta_right_dist) >= SMALL_WALL_LEN / 2:
            self.state = ParkingState.SHIMMY_RIGHT
            self.state_update()
            return
        self.move(SLOW_SPEED, 0.0)

    def shimmy_right(self):
        if time.time() - self.start_time <= SHIMMY_RIGHT_TIME:
            self.state = ParkingState.SHIMMY_LEFT
            self.state_update()
            return
        self.move(-SLOW_SPEED, TURN_RATE)

    def shimmy_left(self):
        if time.time() - self.start_time <= SHIMMY_RIGHT_TIME:
            self.state = ParkingState.SHIMMY_FORWARD
            self.state_update()
            return
        self.move(-SLOW_SPEED, -TURN_RATE)
        
    def shimmy_forward(self):
        if self.right_dist <= SHIMMY_DONE_DIST:
            self.state = ParkingState.DONE
            self.state_update()
            return
        if time.time() - self.start_time <= SHIMMY_RIGHT_TIME:
            self.state = ParkingState.SHIMMY_RIGHT
            self.state_update()
            return
        self.move(SLOW_SPEED, 0.0)

    def straighten_out(self):
        pass
    
    def done(self):
        self.move(0.0, 0.0)

    def move(self, throttle, angle):
        twist = Twist()
        twist.linear.x = throttle
        twist.angular.z = angle
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    spot_detection = SpotDetectionAndParking()
    rclpy.spin(spot_detection)
    spot_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
