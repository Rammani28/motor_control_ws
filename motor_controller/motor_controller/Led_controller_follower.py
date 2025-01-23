import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO  # https://pypi.org/project/RPi.GPIO/

class LedSubscriberNode(Node):
    def __init__(self):
        super().__init__('led_subscriber') # name of node
        self.subscription = self.create_subscription(msg_type=String, topic='/led_control', callback=self.listener_callback, qos_profile=10) # subscribe topic /led_control using String type, queue size is 10
        self.led_pin = 23 #BCM numbering
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.led_pin, GPIO.OUT)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        if msg.data.lower() == 'on':
            GPIO.output(self.led_pin, GPIO.HIGH)
            self.get_logger().info("led turned on")
        elif msg.data.lower() == 'off':
            GPIO.output(self.led_pin, GPIO.LOW)
            self.get_logger().info("led turned off")
        else:
            self.get_logger().info("invalid command")
        
    def __del__(self):  # destructor for LedSubscriberNode class, dk why
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = LedSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()