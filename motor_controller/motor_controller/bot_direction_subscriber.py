import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# import RPi.GPIO as GPIO  # https://pypi.org/project/RPi.GPIO/
# from math import sin, cos
import serial
from motor_controller.utils import target_wheel_rpm
from motor_controller.motor import Motor

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
            rpms_pico = ser.readline().decode().strip()  # Read and decode
            rpm_vec = [int(rpm/2) for rpm in rpms_pico.split(',')] #is in pwm value 

            print(rpms_pico)
            # if rpms_pico:
            #     self.get_logger().info(f"received rpm from pico: {rpms_pico}")

            # joystick angle converted to rpm pwm for each wheel
            u1, u2, u3 = target_wheel_rpm(theta, 0) # -100 to 100 for comparable pwm
            rpm1, rpm2, rpm3 = rpm_vec
            
            self.motor1.rotate(rpm1, u1)

     
    def __del__(self):  # destructor for motor class, why here?
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