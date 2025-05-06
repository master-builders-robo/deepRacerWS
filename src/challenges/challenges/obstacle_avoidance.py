import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from action_interfaces.action import Obstacle



from geometry_msgs.msg import Twist, Vector3
from action_msgs.msg import GoalInfo, GoalStatus, CancelGoal
import time

from sensor_msgs.msg import Image

import cv2

import os


import math
import glob

from collections import deque

import math
import time
import numpy as np

MAX_VEL = 0.34

class avoidObstacle_actionServer(Node):

    def __init__(self):
        super().__init__('Move_around_Obstacle')
        self._action_server = ActionServer(self, Obstacle,'Serve_Obstacle', execute_callback=self.Obstacle_callback)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def Obstacle_callback(self):



        twist = Twist(
            linear=Vector3(x=vel, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0)
        )
        self.vel_pub.publish(twist)


class avoidObstacle_actionClient(Node):

    def __init__(self):
        super().__init__('Request_Move_towards_goal')
        self._action_client = ActionClient(self, Obstacle,'Move_around')

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Obstacle.Goal()

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.sequence))

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.sequence))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        # Shutdown after receiving a result
        rclpy.shutdown()
