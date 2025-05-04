import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

MAX_TURN = 3.0
MAX_SPEED = 0.16

class Racer(Node):
    def __init__(self):
        super().__init__('racer_lidar_reverse')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.prev_turn = 0.0

    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        # Replace inf with max range
        ranges[np.isinf(ranges)] = msg.range_max

        # Split scan into left, center, right
        num_points = len(ranges)
        left = np.nanmean(ranges[num_points*2//3:])
        center = np.nanmean(ranges[num_points//3:num_points*2//3])
        right = np.nanmean(ranges[:num_points//3])

        # Calculate turn: steer away from closer side
        diff = right - left
        turn = np.clip(diff, -1.0, 1.0) * MAX_TURN

        # Smooth turn
        turn = 0.7 * self.prev_turn + 0.3 * turn
        self.prev_turn = turn

        # Always go in reverse
        twist = Twist()
        twist.linear.x = -MAX_SPEED
        twist.angular.z = turn

        self.get_logger().info(f"Reverse driving | Left: {left:.2f}, Right: {right:.2f}, Turn: {turn:.2f}")
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Racer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
