import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
import numpy as np
import time

from cv_bridge import CvBridge

import os
import math
import glob

from collections import deque

from rclpy.qos import QoSProfile, ReliabilityPolicy


class TargetFinding(Node):
    def __init__(self):
        super().__init__('target_finder')

        # Publishers and Subscribers
        qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.camera_sub = self.create_subscription(Image, 'image_raw', self.camera_callback, qos)

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # OpenCV bridge
        self.bridge = CvBridge()

        # === State Management ===
        self.state = "LINE_FOLLOWING"
        self.target_seen_time = None
        self.TARGET_DRIVE_DURATION = 2.0  # seconds to drive toward green target
        self.TARGET_REACHED_AREA = 1000

        self.get_logger().info("Target Finder node started.")

    def camera_callback(self, msg: Image):

        self.get_logger().info(f"AAAAAAAAAAAAAAAAAAA")
        return

        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # possibly might need to add stricter filter to get proper green thingy


        # HSV bounds for bright green
        lower_green = np.array([35, 80, 80])   # Wider HSV
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # cv2.imwrite(f'/home/user/Desktop/deepRacerWS/src/racer/racer/HELLO!!!!!.png', mask)

        # Check for large enough target
        found_target = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300:
                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w // 2

                img_center = frame.shape[1] // 2
                center_threshold = 0.25 * frame.shape[1]

                self.get_logger().info(f"Green object area={area}, center_x={cx}, image_center={img_center}")

                if abs(cx - img_center) < center_threshold:
                    found_target = True
                    break  # We only need one good target

        twist = Twist()
        current_time = time.time()

        self.get_logger().info("HELLOO11111OOOO")

        if self.state == "LINE_FOLLOWING":
            if found_target:
                self.get_logger().info("[STATE1] Target detected! Switching to TARGET_ACQUIRED")
                self.state = "TARGET_ACQUIRED"
                self.target_seen_time = current_time
            else:
                self.get_logger().info("i am here")
                twist.linear.x = 0.21
                twist.angular.z = 0.0  

        elif self.state == "TARGET_ACQUIRED":
            
            #if current_time - self.target_seen_time < self.TARGET_DRIVE_DURATION:
            #    self.get_logger().info("[ACTION1] Driving toward target")
            #    twist.linear.x = 0.2
            #    twist.angular.z = 0.0
            
            if found_target:
                # Steering based on horizontal offset
                img_center = frame.shape[1] // 2
                error = cx - img_center
                correction = -0.003 * error  # Tune this constant as needed

                self.get_logger().info(f"[ACTION] Steering toward target, error={error}, correction={correction:.3f}, Green object area={area}")

                twist.linear.x = 0.17
                twist.angular.z = correction * 3

                # Stop if very close
                if area > self.TARGET_REACHED_AREA:
                    self.get_logger().info("[STATE] Target reached — stopping")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.state = "TARGET_REACHED"

            else:
                self.get_logger().info("[STATE2] Target finished — returning to LINE_FOLLOWING")
                self.state = "LINE_FOLLOWING"
            '''
            if current_time - self.target_seen_time < self.TARGET_DRIVE_DURATION:
                self.get_logger().info("[ACTION] Driving toward target")
                twist.linear.x = 0.17
                twist.angular.z = 0.0
            elif found_target and area > 340:  # Replace 1600 with what you saw in logs
                self.get_logger().info("[STATE] Target reached — stopping")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.state = "TARGET_REACHED"
            else:
                self.get_logger().info("[STATE] Target finished — returning to LINE_FOLLOWING")
                self.state = "LINE_FOLLOWING"
            
            if found_target:
                # Steering based on horizontal offset
                img_center = frame.shape[1] // 2
                error = cx - img_center
                correction = -0.003 * error  # Tune this constant as needed

                self.get_logger().info(f"[ACTION1.0] Steering toward target, error={error}, correction={correction:.3f}")

                twist.linear.x = 0.17
                twist.angular.z = correction

                # Stop if very close
                if area > self.TARGET_REACHED_AREA:
                    self.get_logger().info("[STATE] Target reached — stopping")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.state = "TARGET_REACHED"
            else:
                # Lost sight of target — back to LINE_FOLLOWING
                self.get_logger().info("[STATE] Lost target — returning to LINE_FOLLOWING")
                self.state = "LINE_FOLLOWING"
            '''
        elif self.state == "TARGET_REACHED":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("[STATE3] Holding position at target")


        self.vel_pub.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    node = TargetFinding()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
