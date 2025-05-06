import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data   # KEEP_LAST, depth=1, best‑effort


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
# MAX_SPEED = 0.0
# works
MAX_SPEED = 0.16

class Racer(Node):
    def __init__(self):
        super().__init__('racer')
        self.camera_sub = self.create_subscription(Image, 'image_raw', self.camera_callback, qos_profile=qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.steer = 0.0
        self.written = False
        self.frames = 0
        self.currBlueInd = 0
        self.lastTime = time.time()
        self.numImages = 0
        self.trackFileNames = glob.glob('/home/user/Desktop/deepRacerWS/src/racer/racer/track*.png')
        self.last_process_time = time.time()
        # self.process_interval = 0.05/2/2  # seconds (i.e. 5 Hz)
        self.process_interval = 0.2
        self.prev_turn = 0.0
        self.last_big_change_time = time.time()
        self.doneCooldown = False


        self.turn_history = deque(maxlen=20)  # add this in __init__
        
        self.cooldown_frames = 0  # cooldown counter
        self.COOLDOWN_DURATION = 10  # frames to suppress after big jump
        self.twist = Twist()



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

    def publishToPCA(self, speed, turn=None):
        self.twist.linear.x = speed
        if turn is not None:
            self.twist.angular.z = turn
        self.vel_pub.publish(self.twist)


    def camera_callback(self, msg: Image):

        try:

            image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))


            image = image[:, :-3, :]  # remove 3-pixel strip
            hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

            # cv2.imwrite(f'/home/user/Desktop/deepRacerWS/src/racer/racer/failimg.png', hsv)

            # Downscale to half size (320x240)
            hsv = cv2.resize(hsv, (hsv.shape[1] // 2, hsv.shape[0] // 2), interpolation=cv2.INTER_AREA)

            # STRICT THRESHOLD
            strict_lower = np.array([100, 128, 65])
            # strict_lower = np.array([100, 140, 110])
            strict_upper = np.array([120, 255, 255])
            strict_mask = cv2.inRange(hsv, strict_lower, strict_upper)

            contours, _ = cv2.findContours(strict_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            strict_mask[:] = 0
            for cnt in contours:
                if cv2.contourArea(cnt) > 75:  # scaled area threshold (was 300)
                    cv2.drawContours(strict_mask, [cnt], -1, 255, thickness=cv2.FILLED)

            # point = self.bfs(strict_mask, (strict_mask.shape[0] - 1, strict_mask.shape[1] // 2))

            # Get any valid pixel from strict_mask (white == 255)
            pts = np.argwhere(strict_mask == 255)
            if pts.size == 0:
                self.currBlueInd += 1
                self.publishToPCA(MAX_SPEED)
                return
            point = tuple(pts[pts[:, 0].argmax()])



            if point is None:
                self.currBlueInd += 1
                self.publishToPCA(MAX_SPEED)
                return

            # BROADER THRESHOLD
            broad_lower = np.array([90, 35, 90])
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

            
            # crop_start = int(0.15 * cleaned.shape[0])
            # cleaned = cleaned[crop_start:, :]


            # return

            # === Vectorized turn calculation ===
            ys, xs = np.nonzero(cleaned)
            lowest_y = np.max(ys)
            height = cleaned.shape[0]
            # if lowest_y <= height * 0.50:
            #     # It's near the bottom → maybe bad
            #     self.publishToPCA(MAX_SPEED)
            #     return  # or handle differently
            if xs.size == 0:
                self.publishToPCA(MAX_SPEED)
                return

            # # Skip if too few white pixels (noise)
            # total_pixels = cleaned.shape[0] * cleaned.shape[1]
            # white_ratio = len(xs) / total_pixels
            # if white_ratio < 0.01:
            #     return

            white_pixel_count = len(xs)
            total_pixels = cleaned.shape[0] * cleaned.shape[1]
            if white_pixel_count > 0.8 * total_pixels:
                self.publishToPCA(MAX_SPEED)
                return
            now = time.time()
            # self.get_logger().info(f"P{now - self.last_process_time}")

            if now - self.last_process_time > self.process_interval:
                # cv2.imwrite(f'/home/user/Desktop/deepRacerWS/src/racer/racer/DONE.png', cleaned)
                self.last_process_time = now

            


            half_width = cleaned.shape[1] // 2
            height = cleaned.shape[0]
    
            # Check if the white pixels span more than 40% across the x-axis
            x_span = np.max(xs) - np.min(xs)
            span_ratio = x_span / cleaned.shape[1]
            turn = None
            ys, xs = np.nonzero(cleaned)
            if xs.size == 0:
                # self.get_logger().error(f"CLEANED HAS NOTHING")
                self.publishToPCA(MAX_SPEED)
                return
            # if True:
            # if False:
            if span_ratio < 0.55:
                # self.get_logger().info(f"PROBABLY not TURNING --- ! --- ! --- ! --- ! --- !: {span_ratio:.2f}")
                # You can handle the case here, e.g. ignore, slow down, etc.
                # return


                half_width = cleaned.shape[1] // 2

                center_x = np.mean(xs)
                
                # turn = ((center_x - half_width) / half_width) * MAX_TURN
                err = (center_x - half_width) / half_width  # normalized [-1, 1]


                # --- right after you compute err ---
                DEADZONE = 0.05          # no steering while |err| < 5 %
                GAIN     = 0.6           # P‑gain for the rest

                err = (center_x - half_width) / half_width  # −1 … 1

                if abs(err) < DEADZONE:
                    turn = 0.0
                else:
                    # re‑scale so you still get full range after the dead‑zone
                    scaled = (abs(err) - DEADZONE) / (1 - DEADZONE)
                    turn = math.copysign(scaled, err) * MAX_TURN * GAIN



                # turn = err * MAX_TURN * 0.7  # 0.7 = proportional gain

                # turn = math.copysign(abs(err) ** 0.7, err) * MAX_TURN * 0.7

                self.publishToPCA(MAX_SPEED, turn)
                return
            # self.get_logger().info(f"SEEMS TO BE TURNING")
            

            # ------------------------------------------------------------------------------ TURNING CODE BELOW
        
         

            height = cleaned.shape[0]
            half_width = cleaned.shape[1] // 2

            # Normalize horizontal positions to [-1, 1]
            horizontal_factor = (xs - half_width) / half_width  # stays in [-1, 1]

            # Exponentially emphasize pixels far from center
            horizontal_weights = np.abs(horizontal_factor) ** 3.7

            # Give priority to pixels higher up in the image
            vertical_weights = (1 - (ys / height)) ** 3.0

            # Combine weights
            weights = horizontal_weights * vertical_weights
            weights = np.clip(weights, 0.0, 1.0)

            if np.sum(weights) == 0:
                # self.get_logger().error(f"WEIGHTS ARE ALL ZERO IN THE TURNING CODE")
                self.publishToPCA(MAX_SPEED)
                return

            # Final turn (naturally in [-1, 1])
            turn = np.average(horizontal_factor, weights=weights) * MAX_TURN


            #  ----------------------------------------------------------- START OLD TURN DAMPENING CODE

            # THRESHOLD = 0.07 * MAX_TURN
            # COOLDOWN_TIME = 0.8  # seconds
            # now = time.time()

            # delta = abs(turn - self.prev_turn)

            # if delta >= THRESHOLD:
            #     # self.get_logger().info(f"-----------YES ABOVE THRESHOLD")
            #     if (now - self.last_big_change_time) > COOLDOWN_TIME or self.doneCooldown == False:
            #         self.doneCooldown = True
            #         # self.get_logger().info(f"COOLDOWN TIME REACHED")

            #         # Big change and cooldown passed: commit it and reset
            #         self.last_big_turn = turn
                    
            #         self.turn_history.clear()
            #         # self.turn_history.append(turn)
            #         self.last_big_change_time = now
            #     else:
            #         # Big change too soon: treat as minor, use average
            #         self.turn_history.append(turn)
            #         big_turn_weight = 0.1
            #         avg_turn_history = (sum(self.turn_history) / len(self.turn_history)) if self.turn_history else 0.0
            #         turn = (self.last_big_turn * big_turn_weight) + ((1 - big_turn_weight) * avg_turn_history)
            #         # turn = avg_turn_history
            #         # turn = self.last_big_turn

            # else:
            #     # self.get_logger().info(f"!!!!!!!!!!!!!NOT ABOVE THRESHOLD")
            #     self.turn_history.append(turn)

            # self.prev_turn = turn

            # ----------------------------------------------------------------- END OLD TURN DAMPENING CODE

            THRESHOLD = 0.25 * MAX_TURN      # tune after testing
            ALPHA     = 0.7                  # 70 % new, 30 % history

            delta = abs(turn - self.prev_turn)
            if delta < THRESHOLD:            # small change → blend
                avg = sum(self.turn_history)/len(self.turn_history) if self.turn_history else 0.0
                turn = ALPHA*turn + (1-ALPHA)*avg

            self.turn_history.append(turn)   # never clear
            self.prev_turn = turn

            # self.get_logger().info(f"turning to : {turn}")

                  
            # cv2.imwrite(f'/home/user/Desktop/deepRacerWS/src/racer/racer/DONE.png', cleaned)


    # 
    #  basic but kind of working shit was above
    # # 
    #         leftmost = np.min(xs)
    #         rightmost = np.max(xs)

    #         # Which edge has more pull?
    #         dist_left = abs(leftmost - half_width)
    #         dist_right = abs(rightmost - half_width)

    #         if dist_left > dist_right:
    #             edge_x = leftmost
    #         else:
    #             edge_x = rightmost

    #         turn = ((edge_x - half_width) / half_width) * MAX_TURN

            # MIN_TURN = 1.0
            # # Normalized center distances
            # xs_centered = xs - half_width
            # dist_ratio = xs_centered / half_width

            # # Weight edge pixels more
            # weights = np.abs(dist_ratio) ** 2.0
            # offset = np.average(dist_ratio, weights=weights)

            # # Scale turn more aggressively the farther off we are
            # aggression = abs(offset)  # 0 near center, 1 at edge
            # turn = offset * (MIN_TURN + (MAX_TURN - MIN_TURN) * aggression)



            # white_count = np.count_nonzero(cleaned)

            # self.get_logger().info(f"WHITE RATIo: {white_ratio}")


            # seconds = time.localtime().tm_sec

            # self.get_logger().info(f"SECONDS: {seconds}")


            # self.turn_history.append(turn)


            # weights = [0.4, 0.25, 0.2, 0.1, 0.05]  # newest to oldest
            # recent_turns = list(self.turn_history)[-5:]
            # turn = sum(w * t for w, t in zip(weights, reversed(recent_turns)))



            # ys, xs = np.nonzero(cleaned)
            # if xs.size == 0:
            #     return

            # half_width = cleaned.shape[1] // 2
            # height = cleaned.shape[0]

            # # Weight = just the distance from center, exaggerated
            # horizontal_dist = (xs - half_width) / half_width
            # weights = np.sign(horizontal_dist) * (np.abs(horizontal_dist) ** 7)

            # # JUST USE THIS — no vertical weight
            # # turn = -np.mean(weights) * MAX_TURN
            # turn = np.average(horizontal_dist, weights=(np.abs(horizontal_dist) ** 7))
            # turn = max(min(turn, MAX_TURN), -MAX_TURN)



            # self.get_logger().info(f"Mean horiz dist: {np.mean(horizontal_dist)}")
            # self.get_logger().info(f"FINAL TURN (raw): {turn}")




            # combined_weights += weight
            # self.get_logger().info(f"Mean x: {np.mean(xs)}, Half width: {half_width}")

            # turn = -np.mean(x_weights) * MAX_TURN 
            # turn = -(combined_weights/len(xs)) * MAX_TURN

            # turn = sum(self.turn_history) / len(self.turn_history)

            # speed = MAX_SPEED if abs(turn) < 0.3 else MIN_SPEED
            self.publishToPCA(MAX_SPEED, float(turn))
            # twist.linear.x = speed

        except Exception as e:
            self.get_logger().error(f"Callback crash: {repr(e)}")

    #

def main(args=None):
    rclpy.init(args=args)
    node = Racer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
