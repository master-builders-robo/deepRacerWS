import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ForwardUntilWall(Node):
    def __init__(self):
        super().__init__('forward_until_wall')
        self.threshold = 0.5  
        self.speed = 0.2     

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.front_dist = float('inf')
     
        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg: LaserScan):
      
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
      
        idx_center = int((0.0 - angle_min) / angle_inc)
        window = int(math.radians(5.0) / angle_inc)
    
        i_min = max(0, idx_center - window)
        i_max = min(len(msg.ranges) - 1, idx_center + window)
        front_ranges = msg.ranges[i_min:i_max+1]
     
        valid = [r for r in front_ranges if r > msg.range_min and r < msg.range_max]
        if valid:
            self.front_dist = min(valid)
        else:
            self.front_dist = float('inf')

    def timer_callback(self):
        twist = Twist()
        if self.front_dist > self.threshold:
            twist.linear.x = self.speed
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Wall detected at %.2fm – stopping.' % self.front_dist)
       
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ForwardUntilWall()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
