import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO  # https://pypi.org/project/RPi.GPIO/
from math import sin, cos
import serial

ser = serial.Serial("/dev/serial0", baudrate=115200, timeout=1)


class PwmSubscriberNode(Node):
    def __init__(self):
        super().__init__('bot_direction_subscriber') # name of node
        print("subscriber initialized")
        self.subscription = self.create_subscription(msg_type=String, topic='/bot_direction', callback=self.listener_callback, qos_profile=10) # subscribe topic /led_control using String type, queue size is 10  
        # self.in1 = 12 # physical pin 32 
        # self.in2 = 13 #physical pin 33 
        self.in1 = 5 # physical pin 
        self.in2 = 6 #physical pin  

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        # GPIO.setup(self.led_pin, GPIO.OUT)

        # Initialize hardware PWM for motor 1
        self.pwm_in1 = GPIO.PWM(self.in1, 2000)  # PWM on GPIO 18, frequency=1kHz
        self.pwm_in2 = GPIO.PWM(self.in2, 2000)  # PWM on GPIO 19, frequency=1kHz


        # Start PWM with 0% duty cycle (off)
        self.pwm_in1.start(0)
        self.pwm_in2.start(0)

        
    def required_motor_rpm(self, r:float, d:float, theta:float, vbx:float, vby:float) -> tuple:
        """return u1, u2, u3, ie. how fast each wheel must rotate to move the robot linearly in any direction"""
        u1 = (-d*theta + vbx) / r # how fast wheel 1 must rotate
        u2 = (-d*theta - 0.5*vbx - 0.866*vby) / r
        u3 = (-d*theta - 0.5*vbx + 0.866*vby) / r
        return u1, u2, u3 # return the rpm of each wheel


    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

        rpm_from_pico = ser.readline().decode().strip()  # Read and decode
        if data:
            self.get_logger().info(f"received rpm from pico: {data}")

        theta = float(msg.data) # wbz
        r = 5 # TODO: changel latercm
        d = 15 # TODO: changel later
        v = 200 # say 200 rpm constant 
        vbx = sin(theta) * v
        vby = cos(theta) * v
        u1, u2, u3 = self.required_motor_rpm(r, d, theta, vbx, vby) # target rpm or say target pwm

        # Convert u1, u2, u3 from rpm to pwm duty cycle
        # self.pwm_in1.ChangeDutyCycle(u1/200 * 100) # 200 is max rpm, map to 100% duty cycle
        # self.pwm_in2.ChangeDutyCycle(u1/2) #

        # move motors based on the angle received from joystick
        pwm_value = 20
        if theta != 501:  # 501 means no cmd from joystick
            if theta > 0 and theta < 180:
                self.pwm_in1.ChangeDutyCycle(pwm_value)  # 100% duty cycle on in1 or forward pin
                self.pwm_in2.ChangeDutyCycle(0)   # Turn off reverse pin
                self.get_logger().info("motor rotating forward")
            elif theta  >= 180 and theta < 360:
                self.pwm_in1.ChangeDutyCycle(0) # turn off forward pin
                self.pwm_in2.ChangeDutyCycle(pwm_value)   #turn on reverse pin
                self.get_logger().info("motor rotating forward")
            

        # if msg.data.lower() == 'on':
        #     self.pwm_in1.ChangeDutyCycle(50)  # Example: 50% duty cycle
        #     self.pwm_in2.ChangeDutyCycle(0)   # Turn off reverse pin
        #     self.get_logger().info("motor rotating forward")

        # elif msg.data.lower() == 'off':
        #     # GPIO.output(self.led_pin, GPIO.LOW)
        #     self.pwm_in1.ChangeDutyCycle(0)  
        #     self.pwm_in2.ChangeDutyCycle(50)  
        #     self.get_logger().info("motor rotating reverse")

        # else:
        #     self.pwm_in1.ChangeDutyCycle(0)
        #     self.pwm_in2.ChangeDutyCycle(0)
        #     self.get_logger().info("Invalid command - Motor stopped")
        # self.get_logger().info(f"angle={msg.data}")
        
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