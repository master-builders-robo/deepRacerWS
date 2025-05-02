#!/usr/bin/env python3
import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge


class TargetFinder(Node):
    def __init__(self):
        super().__init__('target_finder')

        # --- parameters ---
        self.camera_topic = self.declare_parameter('camera_topic', 'image_raw').value
        self.lidar_topic = self.declare_parameter('lidar_topic', 'scan').value
        self.target_frame = self.declare_parameter('target_frame', 'base_link').value

        # horizontal field of view in degrees
        hfov_deg = self.declare_parameter('camera_hfov_deg', 60.0).value
        self.hfov = math.radians(hfov_deg)

        # neon‑green HSV range: tweak these for your exact paint/paper
        hsv_lo = self.declare_parameter('color_lower_hsv', [40, 150, 150]).value
        hsv_hi = self.declare_parameter('color_upper_hsv', [80, 255, 255]).value
        self.color_lower = np.array(hsv_lo, dtype=np.uint8)
        self.color_upper = np.array(hsv_hi, dtype=np.uint8)

        # --- state ---
        self.bridge = CvBridge()
        self.latest_scan = None

        # --- subscriptions & publisher ---
        self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.create_subscription(LaserScan, self.lidar_topic, self.lidar_callback, 10)
        self.pub = self.create_publisher(PointStamped, 'target_position', 10)

        self.get_logger().info(f'TargetFinder started: listening on "{self.camera_topic}" & "{self.lidar_topic}"')

    def lidar_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def image_callback(self, img_msg: Image):
        # 1) convert to OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        # 2) HSV threshold
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)

        # 3) find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:  # skip small blobs
                continue

            # approximate polygon and look for quadrilaterals
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            if len(approx) != 4:
                continue

            # compute centroid in pixel coords
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # compute angle of that pixel relative to image center
            angle = ( (cx / w) - 0.5 ) * self.hfov  # radians

            # get LIDAR distance at that angle
            r = self.get_lidar_range(angle)
            if r is None:
                continue

            # convert to vehicle frame (x forward, y left)
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # publish
            pt = PointStamped()
            pt.header = img_msg.header
            pt.header.frame_id = self.target_frame
            pt.point.x = float(x)
            pt.point.y = float(y)
            pt.point.z = 0.0
            self.pub.publish(pt)
            self.get_logger().info(f'Detected square at [x={x:.2f}, y={y:.2f}] (r={r:.2f} m, θ={math.degrees(angle):.1f}°)')

            # draw & label
            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"({x:.2f},{y:.2f})m", (cx + 5, cy - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # show feedback
        cv2.imshow('Neon‑Green Square Finder', frame)
        cv2.waitKey(1)

    def get_lidar_range(self, angle_rad: float):
        """ Map an image angle to the nearest LIDAR beam and return its range. """
        if self.latest_scan is None:
            return None
        idx = int((angle_rad - self.latest_scan.angle_min) / self.latest_scan.angle_increment)
        if 0 <= idx < len(self.latest_scan.ranges):
            r = self.latest_scan.ranges[idx]
            # ignore spurious zero/NaN
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
