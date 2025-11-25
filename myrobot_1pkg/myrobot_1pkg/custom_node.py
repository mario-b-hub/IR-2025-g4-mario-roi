import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('custom_node')
        self.counter_ = 0
        self.timer_ = self.create_timer(0.2, self.move_robot)

        self.subscriber_ = self.create_subscription(
            LaserScan, '/scan', self.scan_callback,10)

    def scan_callback(self, msg):
        ranges = msg.ranges
        # Example: read front, left, and right distances
        front = msg.ranges[0]
        left = msg.ranges[90]
        right = msg.ranges[270]
        
    self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_robot(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_publisher_.publish(msg)
        self.counter_ += 1

    if front < 0.5:
        self.move_robot(0.0, 0.3) # Turn
    else:
        self.move_robot(0.2, 0.0) # Move forward

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
     main()

