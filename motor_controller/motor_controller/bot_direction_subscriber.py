import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from motor_controller.utils import target_wheel_rpm
from motor_controller.motor import Motor

ser = serial.Serial("/dev/serial0", baudrate=115200, timeout=0)


class PwmSubscriberNode(Node):
    def __init__(self):
        super().__init__('bot_direction_subscriber') # name of node
        print("subscriber initialized")
        self.subscription = self.create_subscription(msg_type=String, topic='/bot_direction', callback=self.listener_callback, qos_profile=10) # subscribe topic /led_control using String type, queue size is 10
        self.motor1 = Motor(in1=5, in2=6)


    def listener_callback(self, msg):
        subscribed_msg = msg.data
        theta = float(msg.data.split(',')[0]) 
        wbz = float(msg.data.split(',')[-1])
        # self.get_logger().info(f"Received theta:{theta}, wbz:{wbz}")
        

        if theta == 501:
            self.motor1.stop()
            print("no input from joystick....")
        else:
            rpms_pico = ser.readline().decode().strip()  # Read and decode
            # print(f"Raw data from serial: {rpms_pico}")  # Debugging: Print raw data
            if rpms_pico:
                rpm_vec = [float(rpm) for rpm in rpms_pico.split(',')] #is in pwm value 
                u1, u2, u3 = target_wheel_rpm(theta, wbz) # -100 to 100 for comparable pwm
                print(f"theta:{theta}, wbz:{wbz}")
                # print(f"u1={u1},\t u2={u2},\t u3={u3}")
                rpm1, rpm2, rpm3 = rpm_vec
                # print(f"r1={rpm1},\t r2={rpm2},\t r3={rpm3}")
                print(f"u1:{u1:.1f}")
                self.motor1.rotate(rpm_n=rpm1, u_n=u1)

     
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