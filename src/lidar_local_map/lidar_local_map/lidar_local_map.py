import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import math
import numpy as np

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_local_map')
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)

        # Init Variables
        self.initialized: bool = False
        self.ranges: np.ndarray[float] | None = None
        self.angles: np.ndarray[float] | None = None
        self.angle_min: float | None          = None
        self.angle_max: float | None          = None
        self.angle_increment: float | None    = None

    def lidar_callback(self, msg: LaserScan):
        # Init Variables
        if not self.initialized:
            self.initialized = True

            # Init Consts
            self.angle_min = msg.angle_min
            self.angle_max = msg.angle_max
            self.angle_increment = msg.angle_increment

            # Init Ranges
            self.ranges = np.array(msg.ranges)

            # Create angles
            diff = self.angle_max - self.angle_min
            num = int(diff / self.angle_increment)
            self.angles = np.linspace(start=self.angle_min, stop=self.angle_max, num=num)
            return
        
        # Update ranges
        for i, msg_range in enumerate(msg.ranges):
            if math.isfinite(msg_range):
                self.ranges[i] = msg_range

    def angle_to_index(self, angle: float) -> int | None:
        if not self.initialized:
            return None
        # Clamp val
        new_angle = max(min(angle, np.pi), -np.pi)
        new_angle += np.pi
        index = int(new_angle / self.angle_increment)
        return index % len(self.ranges)

    def get_ray(self, angle: float) -> tuple | None:
        if not self.initialized:
            return None
        index = self.angle_to_index(angle)
        return (self.ranges[index], self.angles[index])

    # def get_rays(self, min_angle: float, max_angle: float) -> tuple | None:
    #     if not self.initialized:
    #         return None
    #     min_index = self.angle_to_index(min_angle)
    #     max_index = (self.angle_to_index(max_angle) + 1) % len(self.ranges)
    #     self.get_logger().info(f'min_index: {min_index}, max_index: {max_index}')
    #     return (self.ranges[min_index:max_index], self.angles[min_index:max_index])
    
    # def get_dist(self, angle_in: float, amount_of_rays: int = 1) -> float | None:
    #     if not self.initialized:
    #         return None
    #     min_angle = angle_in - self.angle_increment*((amount_of_rays - 1) // 2)
    #     max_angle = angle_in + self.angle_increment*((amount_of_rays - 1) // 2)

    #     dists, angles = self.get_rays(min_angle, max_angle)

    #     total = 0.0
    #     for dist, angle in zip(dists, angles):
    #         angle_diff = angle_in - angle
    #         self.get_logger().info(f'min_angle: {min_angle} angle: {angle}')
    #         total += np.cos(angle_diff) * dist

    #     return total / len(self.ranges)

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()