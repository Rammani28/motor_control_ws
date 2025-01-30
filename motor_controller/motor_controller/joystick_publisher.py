from sensor_msgs.msg import Joy
# from custom_msgs.msg import MotorControl
from rclpy.node import Node
import rclpy

class JoystickNode(Node):
    def __init(self):
        super().init('joystick_node')
        self.subscription = self.create_subscription(msg_type=Joy, topic='/joy', callback=self.joy_callback, qos_profile=10)
        # self.publisher = self.create_publisher(MotorControl, '/motor_control', 10)

    def joy_callback(self, msg):
        motor_msg = MotorControl()
        if msg.buttons[4]:  # L1 button
            motor_msg.pwm_value = abs(msg.axes[1]) * 100.0  # Map -1.0 to 1.0 to 0–100%
            motor_msg.direction = 1 if msg.axes[1] > 0 else -1
        else:
            motor_msg.pwm_value = 0.0
            motor_msg.direction = 0
        self.publisher.publish(motor_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    