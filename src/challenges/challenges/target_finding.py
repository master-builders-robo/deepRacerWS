import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode

class TargetFinder(Node):
    def __init__(self):
        super().__init__('target_finder')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',   
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            PointStamped,
            'target_position',
            10)
        self.br = CvBridge()
        self.get_logger().info('TargetFinder node started, waiting for images...')

    def image_callback(self, msg: Image):
        # convert ROS Image to OpenCV BGR image
        cv_img = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # detect all QR codes in the frame
        detections = decode(cv_img)
        for det in detections:
            x, y, w, h = det.rect
            cx, cy = x + w/2, y + h/2

            # publish the center pixel as a PointStamped
            pt = PointStamped()
            pt.header = msg.header
            pt.point.x = float(cx)
            pt.point.y = float(cy)
            pt.point.z = 0.0
            self.publisher.publish(pt)

            # log what we saw
            data = det.data.decode('utf-8')
            self.get_logger().info(
                f"Detected QR '{data}' at pixel ({cx:.1f}, {cy:.1f})")

            # draw a rectangle and center dot on the image for visualization
            cv2.rectangle(cv_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(cv_img, (int(cx), int(cy)), 4, (0, 0, 255), -1)
            cv2.putText(cv_img, data, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # show the annotated image
        cv2.imshow('Target Finder', cv_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TargetFinder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, LaserScan
# from geometry_msgs.msg import PointStamped
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from pyzbar.pyzbar import decode

# class TargetFinder(Node):
#     def __init__(self):
#         super().__init__('target_finder')
#         # Parameters: adjust as needed
#         self.declare_parameter('camera_topic', 'image_raw')
#         self.declare_parameter('lidar_topic', 'scan')
#         self.declare_parameter('target_frame', 'base_link')
#         self.declare_parameter('camera_hfov_deg', 60.0)  # Horizontal field of view
#         self.declare_parameter('color_lower_hsv', [0, 100, 100])   # e.g., red lower
#         self.declare_parameter('color_upper_hsv', [10, 255, 255])  # e.g., red upper

#         self.bridge = CvBridge()
#         self.scan = None

#         self.camera_sub = self.create_subscription(
#             Image,
#             self.get_parameter('camera_topic').get_parameter_value().string_value,
#             self.image_callback, 10)
#         self.lidar_sub = self.create_subscription(
#             LaserScan,
#             self.get_parameter('lidar_topic').get_parameter_value().string_value,
#             self.lidar_callback, 10)
#         self.target_pub = self.create_publisher(PointStamped, 'target_position', 10)

#         self.hfov = np.deg2rad(self.get_parameter('camera_hfov_deg').get_parameter_value().double_value)

#         hsv_lo = self.get_parameter('color_lower_hsv').get_parameter_value().integer_array_value
#         hsv_hi = self.get_parameter('color_upper_hsv').get_parameter_value().integer_array_value
#         self.color_lower = np.array(hsv_lo, dtype=np.uint8)
#         self.color_upper = np.array(hsv_hi, dtype=np.uint8)

#         self.get_logger().info('TargetFinder node started')

#     def lidar_callback(self, msg: LaserScan):
#         self.scan = msg

#     def image_callback(self, img_msg: Image):
#         # Convert ROS Image to OpenCV BGR
#         cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
#         h, w = cv_img.shape[:2]

#         # 1) QR code detection
#         qr_codes = decode(cv_img)
#         if qr_codes:
#             for qr in qr_codes:
#                 (x, y, w_box, h_box) = qr.rect
#                 cx_pix = x + w_box/2
#                 angle = (cx_pix/w - 0.5) * self.hfov  # radians
#                 dist = self.get_lidar_distance(angle)
#                 if dist and self.is_target(dist):
#                     pt = PointStamped()
#                     pt.header = img_msg.header
#                     pt.point.x = dist * np.cos(angle)
#                     pt.point.y = dist * np.sin(angle)
#                     pt.point.z = 0.0
#                     self.target_pub.publish(pt)
#                     self.get_logger().info(f'QR target at {pt.point.x:.2f}, {pt.point.y:.2f}')
#                     return

#         # 2) Color segmentation fallback
#         hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
#         mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         if contours:
#             # choose largest contour
#             c = max(contours, key=cv2.contourArea)
#             M = cv2.moments(c)
#             if M['m00'] > 0:
#                 cx_pix = M['m10']/M['m00']
#                 angle = (cx_pix/w - 0.5) * self.hfov
#                 dist = self.get_lidar_distance(angle)
#                 if dist and self.is_target(dist):
#                     pt = PointStamped()
#                     pt.header = img_msg.header
#                     pt.point.x = dist * np.cos(angle)
#                     pt.point.y = dist * np.sin(angle)
#                     pt.point.z = 0.0
#                     self.target_pub.publish(pt)
#                     self.get_logger().info(f'Color target at {pt.point.x:.2f}, {pt.point.y:.2f}')
#                     return

#     def get_lidar_distance(self, angle_rad):
#         """
#         Map camera angle to LIDAR index and return range.
#         If no LIDAR data, or index out of bounds, returns None.
#         """
#         if not self.scan:
#             return None
#         # compute index
#         idx = int((angle_rad - self.scan.angle_min) / self.scan.angle_increment)
#         if 0 <= idx < len(self.scan.ranges):
#             return self.scan.ranges[idx]
#         return None

#     def is_target(self, distance):
#         """
#         Assume targets are 'invisible' to LIDAR (i.e., range == max_range or no return)
#         Obstacles return valid shorter ranges.
#         """
#         if distance >= self.scan.range_max * 0.97 or np.isinf(distance):
#             return True
#         return False

# def main(args=None):
#     rclpy.init(args=args)
#     node = TargetFinder()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
