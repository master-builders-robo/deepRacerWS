import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2

class DriveNode(Node):
    def init(self):
        super().init('drive_node')
        # open camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')
        # publisher
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # timer @ 10 Hz
        self.create_timer(0.1, self.timer_callback)

    def timercallback(self):
        ret, frame = self.cap.read()
        if ret:
            h, w,  = frame.shape
            b, g, r = frame[h//2, w//2]
            self.get_logger().info(f'Center BGR: {b},{g},{r}')
        twist = Twist()
        twist.linear.x = 0.1
        self.pub.publish(twist)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if name == 'main':
    main()