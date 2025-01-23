import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LedPublisherNode(Node):
    def __init__(self):
        super().__init__('led_publisher') #name of 
        self.publisher = self.create_publisher(msg_type=String, topic='/led_control', qos_profile=10) # data of type int, publish to topic /led_control, qos_profile read here https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-profiles
        self.timer = self.create_timer(2.0, self.publish_message) # publish_message callback fumction will be run every 2 second

    def publish_message(self):
        msg = String()
        msg.data = input("Enter LED command (ON/OFF): ")
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg}") #log message or the thing that is printed on console


def main(args=None):
    rclpy.init(args=args)
    node = LedPublisherNode()
    try:
        rclpy.spin(node=node) # keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
