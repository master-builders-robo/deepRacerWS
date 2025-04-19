import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg._laser_scan import LaserScan


class StopInfrontOfWall(Node):

    def __init__(self):
        super().__init__('stop_infront_of_wall')
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.vel_pub = self.create_publisher()

    def lidar_callback(self, msg: LaserScan):
        pass


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()