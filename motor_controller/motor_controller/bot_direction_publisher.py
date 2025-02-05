import math
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import rclpy
from rclpy.node import Node

class BotDirectionPublisher(Node):
    def __init__(self):
        super().__init__('bot_direction_publisher') # node name
        self.publisher = self.create_publisher(msg_type=String, topic='/bot_direction', qos_profile=10) # data of type string, publish to topic /bot_direction, qos_profile read here https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-profiles
        self.subscription = self.create_subscription(msg_type=Joy, topic='/joy', callback=self.joy_callback, qos_profile=10) # subscribe to /Joy node
        self.get_logger().info("LED publisher Node initialized. listening to joystick input...")

    def joy_callback(self, msg):
        """0, 1, 3 are the index of joy sticks in message published by /joy"""
        strafe_x = msg.axes[0] * -1 # multiplied by -1 to make right as +ve and left as -ve(for convience)
        strafe_y = msg.axes[1] # -1 is down, +1 is top, strafe means to move sideways
        rotational_omega = msg.axes[3] * -1 # this is omega bZ or wbz, multiplied by -1 to make right as +ve and left as -ve
        wbz = rotational_omega
        # Calculate the angle in degrees
        # atan2 works fine instead of manually setting angle
        angle = math.degrees(math.atan2(strafe_y, strafe_x)) # atan2 returns angle in radians, convert to degrees
        if angle < 0:
            angle += 360

        message_to_be_published = f"{angle:.2f},{wbz}"
        if strafe_x == 0 and strafe_y == 0 and wbz == 0: # if all values are 0, stop the bot
            message_to_be_published = "501"
        self.publish_message(message_to_be_published) # angle will be from 0 to 360, or 501

    def publish_message(self, msg):
        message = String()
        message.data = msg
        self.publisher.publish(message)
        self.get_logger().info(f"Published: {message.data}") # log message to console


def main(args=None):
    rclpy.init(args=args)
    node = BotDirectionPublisher()
    try:
        rclpy.spin(node=node) # keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()