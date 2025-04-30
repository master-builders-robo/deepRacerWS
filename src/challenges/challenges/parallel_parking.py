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
from enum import Enum

import numpy as np
import time

from util.lidar import Lidar, LidarAngles

SLOW_SPEED = 0.15
SMALL_WALL_LEN = 0.4572
BIG_WALL_LEN = 0.762
MIN_GAP = 0.15
AREA = 0.34
TURN_RATE = 10.0

class ParkingState(Enum):
    SEARCHING = 'searching'
    PARKING_LEFT = 'parking_left'
    PARKING_RIGHT = 'parking_right'
    DONE = 'done'

class SpotDetectionAndParking(Node):
    def __init__(self):
        super().__init__('spot_detection_parking_node')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state: ParkingState = ParkingState.SEARCHING

        self.lidar = Lidar()
        self.front_range = math.nan
        self.back_range  = math.nan
        self.left_range  = math.nan
        self.right_range = math.nan

    def scan_callback(self, msg: LaserScan):
        # Get indicies
        self.lidar.update(msg)
        self.front_range = self.lidar.get_dist(LidarAngles.FRONT_E.value)
        self.back_range = self.lidar.get_dist(LidarAngles.BACK_E.value)
        self.left_range = self.lidar.get_dist(LidarAngles.LEFT_E.value)
        self.right_range = self.lidar.get_dist(LidarAngles.RIGHT_E.value)


    def timer_callback(self):
        twist = Twist()

        self.get_logger().info(f'Current State: {self.state}, Right Range: {self.lidar.get_dist(90)}')
        
        if self.state == ParkingState.SEARCHING:
            # Drive forward looking for an opening
            if self.lidar.get_dist(90) >= SMALL_WALL_LEN:
                self.get_logger().info('Parking spot detected!')
                self.state = ParkingState.PARKING_RIGHT

            twist.linear.x = SLOW_SPEED  # FORWARD
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        elif self.state == ParkingState.PARKING_RIGHT:
            # Drive backward + turn right first
            twist.linear.x = -SLOW_SPEED  # BACKWARD
            twist.angular.z = -TURN_RATE  # RIGHT turn
            self.cmd_vel_pub.publish(twist)

            # After some backing up, when close to back wall, start straightening
            if self.lidar.get_dist(90) <= BIG_WALL_LEN / 2 and self.lidar.get_dist(0) <= BIG_WALL_LEN / 2 and self.lidar.get_dist(-90) <= MIN_GAP:
                time.sleep.robot(0.5)
                self.state = ParkingState.PARKING_LEFT
                self.get_logger().info('Switching to left turn!')

        elif self.state == ParkingState.PARKING_LEFT:
            # Drive backward + turn left to straighten into spot
            twist.linear.x = -SLOW_SPEED
            twist.angular.z = TURN_RATE # LEFT turn
            self.cmd_vel_pub.publish(twist)

            # Done when centered in box
            if (self.front_range < 0.762 and 
                self.back_range < 0.762 and 
                self.right_range < MIN_GAP): 
                self.state = ParkingState.DONE
                self.get_logger().info('Done parking!')


        elif self.state == ParkingState.DONE:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    spot_detection = SpotDetectionAndParking()

    rclpy.spin(spot_detection)

    spot_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
