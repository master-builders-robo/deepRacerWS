import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import numpy as np
import cv2

from collections import deque

MAX_TURN = 3.0
MAX_SPEED = 0.15
MIN_SPEED = 0.135

class Racer(Node):
    def __init__(self):
        super().__init__('racer')
        self.camera_sub = self.create_subscription(Image, 'image_raw', self.camera_callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.steer = 0.0
        self.written = False
        self.frames = 0

    def bfs(self, image, start):
        visited = np.zeros_like(image, dtype=bool)
        q = deque([start])
        visited[start] = True
        while len(q) != 0:
            y, x = q.popleft()
            if image[y, x] == 255:
                return (y, x)
            for dy, dx in [(-1,0),(1,0),(0,-1),(0,1)]:
                ny, nx = y+dy, x+dx
                if 0 <= ny < image.shape[0] and 0 <= nx < image.shape[1]:
                    if not visited[ny, nx]:
                        visited[ny, nx] = True
                        q.append((ny, nx))
        return None

    def camera_callback(self, msg: Image):
        # self.get_logger().info(f'encode: {msg.encoding}')
        # self.get_logger().info(f'size: {len(msg.data)}')
        # self.get_logger().info(f'width: {(msg.width)}')
        # self.get_logger().info(f'height: {(msg.height)}')
        # return
        
        # image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        image = cv2.imread('/home/user/Desktop/deepRacerWS/src/racer/racer/exampleblue.png')
        image = image[:, :-3, :]

        # if not self.written:
        #     cv2.imwrite('/home/user/Desktop/deepRacerWS/src/racer/racer/frame0.png', image) 

        # greyscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # if not self.written:
        #     cv2.imwrite('/home/user/Desktop/deepRacerWS/src/racer/racer/frame1.png', greyscale)    

        # Step 1: Edge detection
        # edges = cv2.Canny(greyscale, 50, 150)

        if not self.written:
            cv2.imwrite('/home/user/Desktop/deepRacerWS/src/racer/racer/frame2.png', image)


        # # Step 2: Fill gaps with closing (dilate then erode)
        # close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
        # dilated = cv2.dilate(edges, close_kernel, iterations=2)
        # closed = cv2.erode(dilated, close_kernel, iterations=1)

        # # Step 3: Thicken result a bit more
        # thick_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
        # thickened = cv2.dilate(closed, thick_kernel, iterations=1)

        # Step 4 (Alternative): Remove blobs by contour area


        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define blue range (tweak if needed)
        # lower_blue = np.array([100, 150, 50])
        # upper_blue = np.array([140, 255, 255])
        lower_blue = np.array([85, 50, 20])    # Includes some cyan and dull blue
        upper_blue = np.array([150, 255, 255]) # Includes bright to deep blue

        # if not self.written:
        #     cv2.imwrite('/home/user/Desktop/deepRacerWS/src/racer/racer/frame3.png', cleaned)

        # Create mask
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask = np.zeros_like(mask)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300:  # Keep only big shapes (tweak this)
                cv2.drawContours(mask, [cnt], -1, 255, thickness=cv2.FILLED)

        cleaned = mask
        

        if not self.written:
            self.written = True
            cv2.imwrite('/home/user/Desktop/deepRacerWS/src/racer/racer/bluedone.png', cleaned)
        return

        start = self.bfs(cleaned, (cleaned.shape[0] - 1, (cleaned.shape[1] - 1) // 2))

        # cleaned: binary image (0 or 255)
        _, labels = cv2.connectedComponents(cleaned)

        # Choose your pixel (x, y)
        cluster_id = labels[start[0], start[1]]  # Note: (row, col) = (y, x)

        # Create a mask for just that cluster
        target_cluster = (labels == cluster_id).astype(np.uint8) * 255
        cleaned = target_cluster

        # Optional save once
        # if not self.written:
        #     self.written = True
        # if self.frames % 5 == 0:
        #     cv2.imwrite('/home/user/Desktop/deepRacerWS/src/racer/racer/frame4.png', cleaned)
        
        self.frames += 1
        
        turn = 0.0
        total = 0.0

        for y, col in enumerate(cleaned):
            for x, pixel in enumerate(col):
                if pixel == 0:
                    continue
                y_weight = (cleaned.shape[0] - y) / cleaned.shape[0]
                half_width = cleaned.shape[1] // 2
                x_weight = (float(half_width - x) / half_width)
                turn -= y_weight * x_weight * MAX_TURN
                total += 1.0
        turn /= total
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
