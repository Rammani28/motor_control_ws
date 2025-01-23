import rclpy  #ros2 client library for writing ros2 applications
from rclpy.node import Node
# from gpiozero import LED
import time


# led = LED(23)


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node') # name =my_node
        self.get_logger().info('Hello ROS 2!') # creates a log entry at INFO level, prints "hello to console"

def main(args=None):
    rclpy.init(args=args) # must include
    node = MyNode()
    print("node started")
    rclpy.spin(node) # keeps node running
    node.destroy_node() #clean up and destroy the node when no longer needed
    print("node destroyed")
    rclpy.shutdown() #shutdown rcl
   

if __name__ == '__main__':
    print("starting main()")
    main()
    