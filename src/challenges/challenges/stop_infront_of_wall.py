import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

import math
import time
import numpy as np

MAX_VEL = 0.34 #concrete floor 0.4
LIDAR_TO_FRONT = 0.1

class StopInfrontOfWall(Node):

    def __init__(self):
        super().__init__('stop_infront_of_wall')
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.samples = []
        self.start_sample_time = 0.0
        self.is_sampling = False

    def lidar_callback(self, msg: LaserScan):
        center_index = len(msg.ranges) // 2
        right_index = (center_index + 1) % len(msg.ranges)
        left_index = center_index - 1

        center_dist = msg.ranges[center_index]
        right_dist = msg.ranges[right_index] * math.cos(msg.angle_increment)
        left_dist = msg.ranges[left_index] * math.cos(msg.angle_increment)
        dists = [center_dist, right_dist, left_dist]
        dist_sum = 0.0
        n = 0
        for dist in dists:
            if not math.isfinite(dist):
                continue
            dist_sum += dist
            n += 1
        if n == 0:
            return
        average_dist = dist_sum / n

        if self.is_sampling:
            if time.time() - self.start_sample_time > 1.0 and len(self.samples) >= 1:
                avg = np.mean(self.samples)
                std = np.std(self.samples)
                if avg - std*0.5 >= 0.5:
                    self.is_sampling = False
                self.start_sample_time = time.time()
            if math.isfinite(average_dist):
                self.samples.append(average_dist)
            return

        t = max(min((average_dist - 0.5 - LIDAR_TO_FRONT)*2.0, 1.0), 0.0)
        assert(t <= 1.0)
        assert(0.0 <= t)

        if t == 0.0:
            self.is_sampling = True
            self.start_sample_time = time.time()

        self.get_logger().info(f't: {t}')

        vel = MAX_VEL * t
        assert(vel <= MAX_VEL)
        assert(0.0 <= vel)
        self.get_logger().info(f'vel: {vel}')

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
