from custom_msgs.msg import MotorControl
import RPi.GPIO as GPIO
from rclpy.node import Node
import rclpy

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(MotorControl, '/motor_control', self.motor_callback, 10)
        GPIO.setmode(GPIO.BCM)
        self.in1 = 12
        self.in2 = 13
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        self.pwm = GPIO.PWM(self.in1, 100)  # 100 Hz PWM frequency
        self.pwm.start(0)

    def motor_callback(self, msg):
        if msg.direction == 1:
            GPIO.output(self.in2, GPIO.LOW)
            self.pwm.ChangeDutyCycle(msg.pwm_value)
        elif msg.direction == -1:
            GPIO.output(self.in2, GPIO.HIGH)
            self.pwm.ChangeDutyCycle(msg.pwm_value)
        else:
            self.pwm.ChangeDutyCycle(0)  # Stop the motor

    def __del__(self):
        self.pwm.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
