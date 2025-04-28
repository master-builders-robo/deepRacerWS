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

SLOW_SPEED = 0.11
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
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.state: ParkingState = ParkingState.PARKING_LEFT
        self.right_range = math.nan
        self.left_range = math.nan
        self.front_range = math.nan
        self.back_range = math.nan

    def scan_callback(self, msg: LaserScan):
        # Get indicies
        right_range = msg.ranges[((len(msg.ranges)) // 4) - 1]
        left_range = msg.ranges[(len(msg.ranges)*3) // 4]
        front_range = msg.ranges[len(msg.ranges) // 2]
        back_range = msg.ranges[0]

        if math.isfinite(right_range):
            self.right_range = right_range
        if math.isfinite(left_range):
            self.left_range = left_range
        if math.isfinite(front_range):
            self.front_range = front_range
        if math.isfinite(back_range):
            self.back_range = back_range


    def timer_callback(self):
        twist = Twist()

        self.get_logger().info(f'Current State: {self.state}, Right Range: {self.right_range}')
        
        if self.state == ParkingState.SEARCHING:
            if self.right_range >= SMALL_WALL_LEN:
                self.get_logger().info('Parking spot detected!')
                self.state = ParkingState.PARKING_LEFT

            twist.linear.x = -SLOW_SPEED  # Drive backward slowly
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

        elif self.state == ParkingState.PARKING_LEFT:
            twist.linear.x = -SLOW_SPEED
            twist.angular.z = TURN_RATE
            self.cmd_vel_pub.publish(twist)

            if self.right_range < MIN_GAP or self.back_range <= MIN_GAP:
                self.state = ParkingState.PARKING_LEFT

        elif self.state == ParkingState.PARKING_RIGHT:
            twist.linear.x = -SLOW_SPEED
            twist.angular.z = -TURN_RATE
            self.cmd_vel_pub.publish(twist)

            if self.back_range <= BIG_WALL_LEN / 2 and self.front_range <= BIG_WALL_LEN / 2 and self.right_range <= MIN_GAP:
                self.state = ParkingState.DONE
                self.get_logger().info('Done!')

        elif self.state == ParkingState.DONE:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SpotDetectionAndParking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

