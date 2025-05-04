import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist, Vector3
from floor_map.action import forwardSquare


import math
import time
import numpy as np

MAX_VEL = 0.34

class avoidObstacle_actionServer(Node):

    def __init__(self):
        super().__init__('Move_towards_goal_unless_Obstacle')
        self._action_server = ActionServer(self, forwardSquare,'lookAhead', execute_callback=self.FreeSpace_callback, callback_group=ReentrantCallbackGroup, goal_callback=self.goal_callback,cancel_callback=self.cancel_callback)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def FreeSpace_callback(self):

            if self.is_sampling:
            if time.time() - self.start_sample_time > 1.0 and len(self.samples) >= 1:
                avg = np.mean(self.samples)
                std = np.std(self.samples)
                if avg - std*0.5 >= 0.5:
                    self.is_sampling = False
                self.start_sample_time = time.time()
            if math.isfinite(average_dist):
                self.samples.append(average_dist)
            return

        t = max(min((average_dist - 0.5 - LIDAR_TO_FRONT)*2.0, 1.0), 0.0)
        assert(t <= 1.0)
        assert(0.0 <= t)

        if t == 0.0:
            self.is_sampling = True
            self.start_sample_time = time.time()

        self.get_logger().info(f't: {t}')

        vel = MAX_VEL * t
        assert(vel <= MAX_VEL)
        assert(0.0 <= vel)
        self.get_logger().info(f'vel: {vel}')

        twist = Twist(
            linear=Vector3(x=vel, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0)
        )
        self.vel_pub.publish(twist)

    def Obstacle_callback(self):

        if self.is_sampling:
            if time.time() - self.start_sample_time > 1.0 and len(self.samples) >= 1:
                avg = np.mean(self.samples)
                std = np.std(self.samples)
                if avg - std*0.5 >= 0.5:
                    self.is_sampling = False
                self.start_sample_time = time.time()
            if math.isfinite(average_dist):
                self.samples.append(average_dist)
            return

        t = max(min((average_dist - 0.5 - LIDAR_TO_FRONT)*2.0, 1.0), 0.0)
        assert(t <= 1.0)
        assert(0.0 <= t)

        if t == 0.0:
            self.is_sampling = True
            self.start_sample_time = time.time()

        self.get_logger().info(f't: {t}')

        vel = MAX_VEL * t
        assert(vel <= MAX_VEL)
        assert(0.0 <= vel)
        self.get_logger().info(f'vel: {vel}')

        twist = Twist(
            linear=Vector3(x=vel, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0)
        )
        self.vel_pub.publish(twist)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT