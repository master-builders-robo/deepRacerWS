import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
import numpy as np
import time

from cv_bridge import CvBridge


class TargetFinder(Node):
    def __init__(self):
        super().__init__('target_finder')

        # Publishers and Subscribers
        self.camera_sub = self.create_subscription(Image, 'image_raw', self.camera_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # OpenCV bridge
        self.bridge = CvBridge()

        # === State Management ===
        self.state = "LINE_FOLLOWING"
        self.target_seen_time = None
        self.TARGET_DRIVE_DURATION = 2.0  # seconds to drive toward green target

        self.get_logger().info("Target Finder node started.")

    def camera_callback(self, msg: Image):
        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # HSV bounds for #00FF00 — very bright green
        lower_green = np.array([50, 240, 240])
        upper_green = np.array([70, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Check for large enough target
        found_target = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 400:
                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w // 2
                cy = y + h // 2

                # Check if near image center (within 25% of width center)
                img_center = frame.shape[1] // 2
                if abs(cx - img_center) < 0.25 * frame.shape[1]:
                    found_target = True
                    break

        twist = Twist()
        current_time = time.time()

        if self.state == "LINE_FOLLOWING":
            if found_target:
                self.get_logger().info("[STATE] Target detected! Switching to TARGET_ACQUIRED")
                self.state = "TARGET_ACQUIRED"
                self.target_seen_time = current_time
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0  # Idle or can insert line following here

        elif self.state == "TARGET_ACQUIRED":
            if current_time - self.target_seen_time < self.TARGET_DRIVE_DURATION:
                self.get_logger().info("[ACTION] Driving toward target")
                twist.linear.x = 0.17
                twist.angular.z = 0.0
            else:
                self.get_logger().info("[STATE] Target finished — returning to LINE_FOLLOWING")
                self.state = "LINE_FOLLOWING"

        self.cmd_vel_pub.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    node = TargetFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
