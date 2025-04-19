import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg._laser_scan import LaserScan
from geometry_msgs.msg import Twist, Vector3


class StopInfrontOfWall(Node):

    def __init__(self):
        super().__init__('stop_infront_of_wall')
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def lidar_callback(self, msg: LaserScan):
        center_index = len(msg.ranges) / 2
        dist_from_wall = msg.ranges[center_index]
        vel = max(min(dist_from_wall - 0.5, 1), 0)
        twist = Twist(Vector3(vel, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        self.vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    stop_infront_of_wall = StopInfrontOfWall()

    rclpy.spin(stop_infront_of_wall)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stop_infront_of_wall.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()