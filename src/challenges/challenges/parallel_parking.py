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

SLOW_SPEED = 0.34

class SpotDetectionAndParking(Node):
    def __init__(self):
        super().__init__('spot_detection_parking_node')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.state = 'searching'
        self.parking_started = False

    def scan_callback(self, msg):
        if self.state == 'searching':
            # Process LIDAR data to find gaps on the right side
            right_ranges = msg.ranges[0:30]  # Assuming 0 is front, right side ~ 0-30 degrees
            gaps = [r for r in right_ranges if r > 0.5]  # 0.5m threshold for a valid gap
            
            if len(gaps) > 20:
                self.get_logger().info('Parking spot detected!')
                self.state = 'parking'
                self.parking_started = True

    def timer_callback(self):
        twist = Twist()
        
        if self.state == 'searching':
            twist.linear.x = -SLOW_SPEED  # Drive backward slowly
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

        elif self.state == 'parking' and self.parking_started:
            self.parallel_parking_routine()
            self.parking_started = False  # Prevent re-triggering

        elif self.state == 'done':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

    def parallel_parking_routine(self):
        self.get_logger().info('Starting parking maneuver...')
        
        # Simple hardcoded parking sequence
        twist = Twist()

        # Step 1: Reverse and curve right
        twist.linear.x = -SLOW_SPEED
        twist.angular.z = 0.3
        self.cmd_vel_pub.publish(twist)
        self.sleep_robot(3.0)

        # Step 2: Reverse and curve left to straighten
        twist.linear.x = -SLOW_SPEED
        twist.angular.z = -0.3
        self.cmd_vel_pub.publish(twist)
        self.sleep_robot(2.5)

        # Step 3: Stop
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        self.state = 'done'
        self.get_logger().info('Parking complete.')

    def sleep_robot(self, duration_sec):
        # Blocking sleep for simplicity (can improve with timers)
        end_time = self.get_clock().now().seconds_nanoseconds()[0] + duration_sec
        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = SpotDetectionAndParking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

