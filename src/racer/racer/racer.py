import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import numpy as np
import cv2
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
        self.written = False

    def camera_callback(self, msg: Image):

        # print("SIZE ", len(msg.data))
        
        self.get_logger().info(f'encode: {msg.encoding}')
        self.get_logger().info(f'size: {len(msg.data)}')
        
        # image = np.array(msg.data).resize((msg.height, msg.width, 3))
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

        greyscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(image, threshold1=50, threshold2=150)

        
        if not self.written:
            self.written = True
            cv2.imwrite('/home/user/Desktop/deepRacerWS/src/racer/racer/frame.png', edges)

        
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
