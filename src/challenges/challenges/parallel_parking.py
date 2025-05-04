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

# distance in meters
SLOW_SPEED = 0.15
SMALL_WALL_LEN = 0.4572  # 18 inches
BIG_WALL_LEN = 0.762     # 30 inches
MIN_GAP = 0.15
TURN_RATE = 10.0

class ParkingState(Enum):
    SEARCHING = 'searching'
    FOUND_FIRST_WALL = 'found_first_wall'
    POTENTIAL_SPOT = 'potential_spot'
    FOUND_SECOND_WALL = 'found_second_wall'
    BACK_TO_SPOT = 'back_to_spot'
    PARKING_RIGHT = 'parking_right'
    PARKING_LEFT = 'parking_left'
    DONE = 'done'

class SpotDetectionAndParking(Node):
    def __init__(self):
        super().__init__('spot_detection_parking_node')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.state: ParkingState = ParkingState.SEARCHING

        self.lidar = Lidar()
        self.front_range = math.nan
        self.back_range  = math.nan
        self.left_range  = math.nan
        self.right_range = math.nan

        self.delay_until = None
        self.found_gap_at = None

    def pause(self, duration_sec):
        self.delay_until = time.time() + duration_sec

    def is_waiting(self):
        return self.delay_until is not None and time.time() < self.delay_until

    def scan_callback(self, msg: LaserScan):
        self.lidar.update(msg)
        self.front_range = self.lidar.get_dist(LidarAngles.FRONT_E.value)
        self.back_range = self.lidar.get_dist(LidarAngles.BACK_E.value)
        self.left_range = self.lidar.get_dist(LidarAngles.LEFT_E.value)
        self.right_range = self.lidar.get_dist(LidarAngles.RIGHT_E.value)

    def timer_callback(self):
        twist = Twist()

        if self.is_waiting():
            self.cmd_vel_pub.publish(Twist())
            return

        right_dist = self.lidar.get_dist(90)
        self.get_logger().info(f"[DEBUG] Right LIDAR distance: {right_dist}")

        if math.isnan(right_dist):
            return

        if self.state == ParkingState.SEARCHING:
            if right_dist < SMALL_WALL_LEN:
                self.state = ParkingState.FOUND_FIRST_WALL
                self.get_logger().info("[STATE] FOUND_FIRST_WALL")
            twist.linear.x = SLOW_SPEED
            self.cmd_vel_pub.publish(twist)

        elif self.state == ParkingState.FOUND_FIRST_WALL:
            if right_dist > SMALL_WALL_LEN:
                self.found_gap_at = time.time()
                self.state = ParkingState.POTENTIAL_SPOT
                self.get_logger().info("[STATE] POTENTIAL_SPOT")
            twist.linear.x = SLOW_SPEED
            self.cmd_vel_pub.publish(twist)

        elif self.state == ParkingState.POTENTIAL_SPOT:
            if right_dist < SMALL_WALL_LEN:
                self.state = ParkingState.FOUND_SECOND_WALL
                self.get_logger().info("[STATE] FOUND_SECOND_WALL")
            twist.linear.x = SLOW_SPEED
            self.cmd_vel_pub.publish(twist)

        elif self.state == ParkingState.FOUND_SECOND_WALL:
            # Give time to move a bit further past the spot
            self.pause(1.0)
            self.state = ParkingState.BACK_TO_SPOT
            self.get_logger().info("[STATE] BACK_TO_SPOT")

        elif self.state == ParkingState.BACK_TO_SPOT:
            if right_dist > 0.3:
                self.state = ParkingState.PARKING_RIGHT
                self.get_logger().info("[STATE] PARKING_RIGHT")
            twist.linear.x = -SLOW_SPEED
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

        elif self.state == ParkingState.PARKING_RIGHT:
            if self.delay_until is None:
                self.get_logger().info("[ACTION] Begin turning right")
                self.pause(0.5)  #potential issue is here, need to change the logic instead of timing do it as distance
                return
            elif self.is_waiting():
                twist.linear.x = -0.14
                twist.angular.z = - TURN_RATE
                self.get_logger().info(f"[PUBLISH] linear.x={twist.linear.x}, angular.z={twist.angular.z}")
                self.cmd_vel_pub.publish(twist)
                return
            else:
                self.get_logger().info("[STATE] Done turning right — switching to PARKING_LEFT")
                self.state = ParkingState.PARKING_LEFT
                return


        elif self.state == ParkingState.PARKING_LEFT:
            twist.linear.x = -0.14
            twist.angular.z = TURN_RATE
            self.cmd_vel_pub.publish(twist)

            if self.back_range < BIG_WALL_LEN and self.right_range < MIN_GAP:
                self.state = ParkingState.DONE
                self.get_logger().info("[STATE] DONE")

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
