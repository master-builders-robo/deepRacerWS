#!/usr/bin/env python3
import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PointStamped, Twist
from cv_bridge import CvBridge


class TargetFinder(Node):
    def __init__(self):
        super().__init__('target_finder')

        # --- parameters ---
        self.camera_topic = self.declare_parameter('camera_topic', 'image_raw').value
        self.lidar_topic = self.declare_parameter('lidar_topic', 'scan').value
        self.target_frame = self.declare_parameter('target_frame', 'base_link').value

        hfov_deg = self.declare_parameter('camera_hfov_deg', 60.0).value
        self.hfov = math.radians(hfov_deg)

        # HSV range for neon‑green squares
        hsv_lo = self.declare_parameter('color_lower_hsv', [40, 150, 150]).value
        hsv_hi = self.declare_parameter('color_upper_hsv', [80, 255, 255]).value
        self.color_lower = np.array(hsv_lo, dtype=np.uint8)
        self.color_upper = np.array(hsv_hi, dtype=np.uint8)

        # control gains & thresholds
        self.declare_parameter('k_lin', 0.2)
        self.declare_parameter('k_ang', 1.0)
        self.declare_parameter('angle_thresh_deg', 5.0)
        self.declare_parameter('reach_dist', 0.15)  # meters

        self.k_lin = self.get_parameter('k_lin').value
        self.k_ang = self.get_parameter('k_ang').value
        self.angle_thresh = math.radians(self.get_parameter('angle_thresh_deg').value)
        self.reach_dist = self.get_parameter('reach_dist').value

        # --- state ---
        self.bridge = CvBridge()
        self.latest_scan = None
        self.current_target = None  # (x, y) in robot frame

        # --- pubs & subs ---
        self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.create_subscription(LaserScan, self.lidar_topic, self.lidar_callback, 10)
        self.pub_pt = self.create_publisher(PointStamped, 'target_position', 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        # control loop at 10 Hz
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('TargetFinder: ready to detect & drive to squares')

    def lidar_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def image_callback(self, img_msg: Image):
        # only pick a new target if none active
        if self.current_target is not None:
            return

        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) < 500:
                continue
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            if len(approx) != 4:
                continue

            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            angle = ((cx / w) - 0.5) * self.hfov
            r = self.get_lidar_range(angle)
            if r is None:
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)
            self.current_target = (x, y)

            # publish detected position
            pt = PointStamped()
            pt.header = img_msg.header
            pt.header.frame_id = self.target_frame
            pt.point.x = float(x)
            pt.point.y = float(y)
            pt.point.z = 0.0
            self.pub_pt.publish(pt)
            self.get_logger().info(f'New target at x={x:.2f}, y={y:.2f}')

            # highlight it
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            break  # only take first square

        cv2.imshow('Finder', frame)
        cv2.waitKey(1)

    def control_loop(self):
        twist = Twist()
        if self.current_target is None:
            # no target: stop
            self.pub_cmd.publish(twist)
            return

        x, y = self.current_target
        dist = math.hypot(x, y)
        angle = math.atan2(y, x)

        # check if reached
        if dist < self.reach_dist:
            self.get_logger().info('Reached square, moving to next...')
            self.current_target = None
            self.pub_cmd.publish(twist)  # zero
            return

        # angular control
        twist.angular.z = self.k_ang * angle
        # linear only when roughly facing target
        if abs(angle) < self.angle_thresh:
            twist.linear.x = min(self.k_lin * dist, 0.2)

        self.pub_cmd.publish(twist)

    def get_lidar_range(self, angle_rad: float):
        if not self.latest_scan:
            return None
        idx = int((angle_rad - self.latest_scan.angle_min) / self.latest_scan.angle_increment)
        if 0 <= idx < len(self.latest_scan.ranges):
            r = self.latest_scan.ranges[idx]
            if r > 0.01 and not math.isinf(r):
                return r
        return None


def main(args=None):
    rclpy.init(args=args)
    node = TargetFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
