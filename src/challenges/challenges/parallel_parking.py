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

SLOW_SPEED = 0.35
SMALL_WALL_LEN = 0.4572
BIG_WALL_LEN = 0.762
MIN_GAP = 0.15
AREA = 0.34
TURN_RATE = 0.0

class ParkingState(Enum):
    SEARCHING = 0
    PARKING = 1
    DONE = 2


class SpotDetectionAndParking(Node):
    def __init__(self):
        super().__init__('spot_detection_parking_node')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.state: ParkingState = ParkingState.SEARCHING
        self.right_range = 0.0
        self.left_range = 0.0
        self.front_range = 0.0
        self.back_range = 0.0

    def scan_callback(self, msg: LaserScan):
        # Get indicies
        right_range = msg.ranges[(len(msg.ranges)*3) // 4]
        left_range = msg.ranges[len(msg.ranges) // 4]
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
        
        if self.state == ParkingState.SEARCHING:
            if self.right_range >= SMALL_WALL_LEN:
                self.get_logger().info('Parking spot detected!')
                self.state = ParkingState.PARKING
            twist.linear.x = SLOW_SPEED  # Drive backward slowly
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

        elif self.state == ParkingState.PARKING:
            self.parallel_parking_routine()

        elif self.state == ParkingState.DONE:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

    def parallel_parking_routine(self):
        # Simple hardcoded parking sequence
        twist = Twist()

        # Turn Left
        if self.right_range >= MIN_GAP and self.back_range > MIN_GAP:
            twist.linear.x = SLOW_SPEED
            twist.angular.z = -TURN_RATE
            self.cmd_vel_pub.publish(twist)
        # Turn Right
        else:
            twist.linear.x = SLOW_SPEED
            twist.angular.z = TURN_RATE
            self.cmd_vel_pub.publish(twist)
        # Parking Complete State
        if (self.back_range <= 60.2 or self.front_range <= 60.2) and self.right_range <= MIN_GAP:
            self.state = ParkingState.DONE
            self.get_logger().info('Done!')

def main(args=None):
    rclpy.init(args=args)
    node = SpotDetectionAndParking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

