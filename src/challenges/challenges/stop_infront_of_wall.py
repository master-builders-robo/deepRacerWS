import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

import math

class StopInfrontOfWall(Node):

    def __init__(self):
        super().__init__('stop_infront_of_wall')
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def lidar_callback(self, msg: LaserScan):
        center_index = 0
        other1_index = (center_index + 1) % len(msg.ranges)
        other2_index = center_index - 1
        indexes = [center_index, other1_index, other2_index]

        angle_inc = msg.angle_increment
        x_components = []
        for ind in indexes:
            dist_from_wall = msg.ranges[center_index]
            x_component = math.cos(angle_inc) * msg.ranges[ind]
            x_components.append(x_component)
        average_dist = sum(x_components) / len(x_components)
        print("AVERAGE DIST IS ", average_dist)
        print("CENTER DISTANCE: ", dist_from_wall)
        vel = max(min(average_dist - 0.5, 0.2), 0.0)

        print(f"Distance: {dist_from_wall:.2f} → Velocity: {vel:.2f}")

        twist = Twist(
            linear=Vector3(x=vel, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0)
        )
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = StopInfrontOfWall()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
