import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO  # https://pypi.org/project/RPi.GPIO/

class PwmSubscriberNode(Node):
    def __init__(self):
        super().__init__('pwm_subscriber') # name of node
        print("subscriber initialized")
        self.subscription = self.create_subscription(msg_type=String, topic='/led_control', callback=self.listener_callback, qos_profile=10) # subscribe topic /led_control using String type, queue size is 10
        # self.led_pin = 23 #BCM numbering      
        self.in1 = 12
        self.in2 = 13

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        # GPIO.setup(self.led_pin, GPIO.OUT)

        # Initialize hardware PWM
        self.pwm_in1 = GPIO.PWM(self.in1, 1000)  # PWM on GPIO 18, frequency=1kHz
        self.pwm_in2 = GPIO.PWM(self.in2, 1000)  # PWM on GPIO 19, frequency=1kHz

        # Start PWM with 0% duty cycle (off)
        self.pwm_in1.start(0)
        self.pwm_in2.start(0)


    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        if msg.data.lower() == 'on':
            self.pwm_in1.ChangeDutyCycle(50)  # Example: 50% duty cycle
            self.pwm_in2.ChangeDutyCycle(0)   # Turn off reverse pin
            self.get_logger().info("motor rotating forward")

        elif msg.data.lower() == 'off':
            # GPIO.output(self.led_pin, GPIO.LOW)
            self.pwm_in1.ChangeDutyCycle(0)  
            self.pwm_in2.ChangeDutyCycle(50)  
            self.get_logger().info("motor rotating reverse")

        else:
            self.pwm_in1.ChangeDutyCycle(0)
            self.pwm_in2.ChangeDutyCycle(0)
            self.get_logger().info("Invalid command - Motor stopped")
        
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