import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import numpy as np

MAX_TURN = 2.5
MAX_SPEED = 0.15
MIN_SPEED = 0.135

prev_turn = 0.0

class Racer(Node):
    def __init__(self):
        super().__init__('racer')
        self.camera_sub = self.create_subscription(Image, 'image_raw', self.camera_callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.steer = 0.0

    def camera_callback(self, msg: Image):
        image_size = msg.width * msg.height
        greyscale = np.zeros((msg.width // 3, msg.height // 3), dtype=float)
        
        assert(image_size % 3 == 0)
        height = msg.height // 3
        width = msg.width // 3
        for i in range(0, image_size, 3):
            y = (i // 3) // height
            x = (i // 3) % width

            r = float(msg.data[i])
            g = float(msg.data[i + 1])
            b = float(msg.data[i + 2])

            brightness = (0.299*r + 0.587*g + 0.114*b) / 255.0

            greyscale[x, y] = brightness
        
        turn = 0.0

        min_vect = np.min(greyscale, axis=0)

        self.get_logger().info(f'{min_vect}')

        twist = Twist()
        twist.angular.z = turn
        twist.linear.x = MAX_SPEED
        self.vel_pub.publish(twist)
        


def main(args=None):
    rclpy.init(args=args)
    node = Racer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
