import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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
        # self.get_logger().info(f"Received: {msg.data}")
        theta = float(msg.data) # wbz

        if theta != 501:
            rpms_pico = ser.readline().decode().strip()  # Read and decode
            print(f"Raw data from serial: {rpms_pico}")  # Debugging: Print raw data
            if rpms_pico:
                rpm_vec = [float(rpm) for rpm in rpms_pico.split(',')] #is in pwm value 
                print(rpm_vec)

                # joystick angle converted to rpm pwm for each wheel
                u1, u2, u3 = target_wheel_rpm(theta, 0) # -100 to 100 for comparable pwm
                print(f"u1={u1}, u2={u2}, u3={u3}")
                rpm1, rpm2, rpm3 = rpm_vec
                print(f"r1={rpm1}, r2={rpm2}, r3={rpm3}")
                self.motor1.rotate(curr_pwm=rpm1, target_pwm=u1)

     
    def __del__(self):  # destructor for motor class, why here?
        self.motor1.destroy()

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