import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO  # https://pypi.org/project/RPi.GPIO/
from math import sin, cos
import serial
from utils import target_wheel_rpm, Motor
PI = 3.14159265359

ser = serial.Serial("/dev/serial0", baudrate=115200, timeout=1)


class PwmSubscriberNode(Node):
    def __init__(self):
        super().__init__('bot_direction_subscriber') # name of node
        print("subscriber initialized")
        self.subscription = self.create_subscription(msg_type=String, topic='/bot_direction', callback=self.listener_callback, qos_profile=10) # subscribe topic /led_control using String type, queue size is 10  

        self.motor1 = Motor(in1=5, in2=6)


    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        theta = float(msg.data) # wbz

        if theta != 501:
            rpm_from_pico = ser.readline().decode().strip()  # Read and decode
            print(rpm_from_pico)
            if rpm_from_pico:
                self.get_logger().info(f"received rpm from pico: {rpm_from_pico}")

            # joystick angle converted to rpm pwm for each wheel
            u1, u2, u3 = target_wheel_rpm(theta, 0) # -100 to 100 for 


            if u1 > 0:
                self.motor1.rotate_clockwise(u1)
            else: 
                self.motor1.rotate_anticlockwise(u1)

            # if u2 > 0:
            #     self.motor2.rotate_clockwise(u2)
            # else: 
            #     self.motor2.rotate_anticlockwise(u2)
            # if u3 > 0:
            #     self.motor3.rotate_clockwise(u3)
            # else: 
            #     self.motor3.rotate_anticlockwise(u3)


    def __del__(self):  # destructor for LedSubscriberNode class, dk why
        self.pwm_in1.stop()
        self.pwm_in2.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = PwmSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()