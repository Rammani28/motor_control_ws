import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy


class LedPublisherNode(Node):
    def __init__(self):
        super().__init__('led_publisher') # node name
        self.publisher = self.create_publisher(msg_type=String, topic='/led_control', qos_profile=10) # data of type int, publish to topic /led_control, qos_profile read here https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-profiles
        # self.timer = self.create_timer(2.0, self.publish_message) # publish_message callback fumction will be run every 2 second
        self.subscription = self.create_subscription(msg_type=Joy, topic='/joy', callback=self.joy_callback, qos_profile=10) # subscribe to /Joy node
        self.get_logger().info("LED publisher Node initialized. listening to joystick input...")

    def joy_callback(self, msg):
        button_on = 0
        button_off = 1

        if msg.buttons[button_on] == 1:
            self.publish_message("ON")
        elif msg.buttons[button_off] == 0:
            self.publish_message("OFF")


    def publish_message(self, on_or_off):
        msg = String()
        msg.data = on_or_off
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.data}") #log message or the thing that is printed on console


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
