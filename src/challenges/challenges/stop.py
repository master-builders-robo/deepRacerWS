import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from enum import Enum, auto

import numpy as np
import time
import cv2

MAX_SPEED = 0.16
NO_SPEED = 0

class FindStop(Node):
    def __init__(self)::
        super().__init__('stop_finder')

        # Publishers and Subscribers
        self.camera_sub = self.create_subscription(Image, 'image_raw', self.camera_callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # OpenCV bridge
        self.bridge = CvBridge()

        # === State Management ===
        self.state = "FINDING_STOP"
        self.target_seen_time = None
        self.TARGET_DRIVE_DURATION = 2.0 
        self.TARGET_REACHED_AREA = 1000

        self.get_logger().info("Stoping node starting")

    def camera_callback(self, msg: Image):
        
        self.get_logger().info(f"Starting camera")
        twist = Twist()

        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get Stop sign image.
        stop_sign = '../resource/stop.png'
        stop_img = cv2.imread(stop_sign)

        stop_gray = cv2.cvtColor(stop_img, cv2.COLOR_BGR2GRAY)

        # Initialize ORB
        orb = cv2.ORB_create(nfeatures=1000)

        # Detect keypoints and descriptors
        kp1, des1 = orb.detectAndCompute(stop_gray, None)
        kp2, des2 = orb.detectAndCompute(frame_gray, None)

        # Brute-force matcher with Hamming distance
        bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        matches = bf.knnMatch(des1, des2, k=2)

        # Apply Lowe's ratio test
        good_matches = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)

        if len(good_matches) > 10:
            self.get_logger().info(f"Stop sign found")
            self.publishToPCA(NO_SPEED)
            time.sleep(3)
            self.publishToPCA(MAX_SPEED)
            time.sleep(3)#TODO: delete this beacuse this is only here to keep going for a little
            self.publishToPCA(NO_SPEED)
            return
        else:
            self.get_logger().info(f"No stop sign found")

    def publishToPCA(self, speed, turn=None):
        self.twist.linear.x = speed
        if turn is not None:
            self.twist.angular.z = turn
        self.vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = FindStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()