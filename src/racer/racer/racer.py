import rclpy
from rclpy.node import Node

import time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import numpy as np
import cv2

import os


import math
import glob

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
        self.currBlueInd = 0
        self.lastTime = time.time()
        self.numImages = 0
        self.trackFileNames = glob.glob('/home/user/Desktop/deepRacerWS/src/racer/racer/track*.png')

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

        image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        cv2.imwrite(f'/home/user/Desktop/deepRacerWS/src/racer/racer/failimg.png', image)

        self.get_logger().info(f'HIIIIIIIIIIII: {"Aaaaa1"}')

        image = image[:, :-3, :]  # remove 3-pixel strip
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # Downscale to half size (320x240)
        hsv = cv2.resize(hsv, (hsv.shape[1] // 2, hsv.shape[0] // 2), interpolation=cv2.INTER_AREA)

        # STRICT THRESHOLD
        strict_lower = np.array([100, 140, 110])
        strict_upper = np.array([120, 255, 255])
        strict_mask = cv2.inRange(hsv, strict_lower, strict_upper)

        contours, _ = cv2.findContours(strict_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        strict_mask[:] = 0
        for cnt in contours:
            if cv2.contourArea(cnt) > 75:  # scaled area threshold (was 300)
                cv2.drawContours(strict_mask, [cnt], -1, 255, thickness=cv2.FILLED)

        # point = self.bfs(strict_mask, (strict_mask.shape[0] - 1, strict_mask.shape[1] // 2))

        self.get_logger().info(f'HIIIIIIIIIIII: {"Aaaaa2"}')

        # Get any valid pixel from strict_mask (white == 255)
        pts = np.argwhere(strict_mask == 255)
        if pts.size == 0:
            self.currBlueInd += 1
            return
        point = tuple(pts[0])  # (y, x)


        if point is None:
            self.currBlueInd += 1
            return

        # BROADER THRESHOLD
        broad_lower = np.array([90, 85, 90])
        broad_upper = np.array([130, 255, 255])
        broad_mask = cv2.inRange(hsv, broad_lower, broad_upper)

        contours, _ = cv2.findContours(broad_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        broad_mask[:] = 0
        for cnt in contours:
            if cv2.contourArea(cnt) > 75:
                cv2.drawContours(broad_mask, [cnt], -1, 255, thickness=cv2.FILLED)

        _, labels = cv2.connectedComponents(broad_mask)
        cluster_id = labels[point[0], point[1]]
        target_cluster = (labels == cluster_id).astype(np.uint8) * 255
        cleaned = target_cluster

        self.get_logger().info(f'HIIIIIIIIIIII: {"Aaaaa"}')
        cv2.imwrite(f'/home/user/Desktop/deepRacerWS/src/racer/racer/failimgcleaned.png', cleaned)
        return

        # === Vectorized turn calculation ===
        ys, xs = np.nonzero(cleaned)
        if xs.size == 0:
            return
        half_width = cleaned.shape[1] // 2
        height = cleaned.shape[0]
        combined_weights = 0
        for i in range(len(xs)):
            x = xs[i]
            y = ys[i]
            y_portion = height - (height - y)/height
            weight = y_portion * (half_width - x) / half_width
            combined_weights += weight
        
        # x_weights = (half_width - xs) / half_width
        # y_weights = (half_width - ys) / half_width
        # turn = -np.mean(x_weights) * MAX_TURN 
        turn = -(combined_weights/len(xs)) * MAX_TURN

        twist = Twist()
        twist.angular.z = turn
        twist.linear.x = MAX_SPEED
        self.vel_pub.publish(twist)

    # def camera_callback(self, msg: Image):
    #     # if self.currBlueInd >= len(self.trackFileNames):
    #     #     return

    #     # self.get_logger().info(f'HIIIIIIIIIIII: {"A"}')

    #     # return
    #     # image = cv2.imread(f'{self.trackFileNames[self.currBlueInd]}')
    #     image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    #     image = image[:, :-3, :]
    #     hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    #     # STRICT THRESHOLD
    #     strict_lower = np.array([100, 140, 110])
    #     strict_upper = np.array([120, 255, 255])
    #     strict_mask = cv2.inRange(hsv, strict_lower, strict_upper)

    #     contours, _ = cv2.findContours(strict_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     strict_mask = np.zeros_like(strict_mask)
    #     for cnt in contours:
    #         if cv2.contourArea(cnt) > 300:
    #             cv2.drawContours(strict_mask, [cnt], -1, 255, thickness=cv2.FILLED)

    #     # Get a valid point from strict_mask
    #     point = self.bfs(strict_mask, (strict_mask.shape[0] - 1, strict_mask.shape[1] // 2))
    #     if point is None:
    #         self.currBlueInd += 1
    #         return

    #     # BROADER THRESHOLD
    #     # broad_lower = np.array([90, 100, 80])
    #     # broad_upper = np.array([125, 255, 255])
    #     # broad_upper = np.array([130, 255, 255])
    #     # broad_lower = np.array([95, 120, 90])

    #     # broad_lower = np.array([90, 80, 60])
    #     # broad_upper = np.array([130, 255, 255])
    #     broad_lower = np.array([90, 85, 90])
    #     broad_upper = np.array([130, 255, 255])


    #     broad_mask = cv2.inRange(hsv, broad_lower, broad_upper)

    #     contours, _ = cv2.findContours(broad_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     broad_mask = np.zeros_like(broad_mask)
    #     for cnt in contours:
    #         if cv2.contourArea(cnt) > 300:
    #             cv2.drawContours(broad_mask, [cnt], -1, 255, thickness=cv2.FILLED)

    #     _, labels = cv2.connectedComponents(broad_mask)
    #     cluster_id = labels[point[0], point[1]]
    #     target_cluster = (labels == cluster_id).astype(np.uint8) * 255

    #     cleaned = target_cluster

    #     # cv2.imwrite(f'/home/user/Desktop/deepRacerWS/src/racer/racer/weDoneBro{self.currBlueInd}.png', target_cluster)
    #     # filename = os.path.splitext(os.path.basename(self.trackFileNames[self.currBlueInd]))[0]
    #     # cv2.imwrite(f'/home/user/Desktop/deepRacerWS/src/racer/racer/DONE.png', target_cluster)
    #     # self.currBlueInd += 1

    #     turn = 0.0
    #     total = 0.0

    #     for y, col in enumerate(cleaned):
    #         for x, pixel in enumerate(col):
    #             if pixel == 0:
    #                 continue
    #             # y_weight = (cleaned.shape[0] - y) / cleaned.shape[0]
    #             half_width = cleaned.shape[1] // 2
    #             x_weight = (float(half_width - x) / half_width)
    #             turn -= x_weight * MAX_TURN
    #             total += 1.0
    #     turn /= total
    #     twist = Twist()
    #     twist.angular.z = turn
    #     twist.linear.x = MAX_SPEED
    #     self.vel_pub.publish(twist)


    def camera_callback1(self, msg: Image):
        
        # image = np.frombuf`fer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        if self.currBlueInd >= len(self.trackFileNames):
            return

        self.get_logger().info(f'HIIIIIIIIIIII: {self.trackFileNames[self.currBlueInd]}')


        image = cv2.imread(f'{self.trackFileNames[self.currBlueInd]}')
        image = image[:, :-3, :]

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define blue range (tweak if needed)
        # lower_blue = np.array([100, 150, 50])
        # upper_blue = np.array([140, 255, 255])
        # lower_blue = np.array([100, 60, 60]) 
        # upper_blue = np.array([130, 255, 255])

        # THESE ARE WHAT WE WILL USE
        # lower_blue = np.array([90, 100, 80])
        # upper_blue = np.array([125, 255, 255])
        lower_blue = np.array([100, 140, 110])
        upper_blue = np.array([120, 255, 255])

        # Create mask
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask = np.zeros_like(mask)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300:  # Keep only big shapes (tweak this)
                cv2.drawContours(mask, [cnt], -1, 255, thickness=cv2.FILLED)

        cleaned = mask
    
        
        self.get_logger().info(f'stuf: {self.currBlueInd}')

        filename = os.path.splitext(os.path.basename(self.trackFileNames[self.currBlueInd]))[0]
        cv2.imwrite(f'/home/user/Desktop/deepRacerWS/src/racer/racer/{filename}DONE.png', cleaned)

        self.currBlueInd += 1

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
