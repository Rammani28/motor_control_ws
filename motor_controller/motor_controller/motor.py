import RPi.GPIO as GPIO # https://pypi.org/project/RPi.GPIO/


class PIDController:
    def __init__(self, Kp=0.5, Ki=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.integral = 0
        self.error  = 0

    def rpm_to_pwm(self, target_rpm):
        pass

    def compute(self, target_rpm, current_rpm):
        self.error = target_rpm - current_rpm
        # self.integral += self.error
        self.integral = max(min(self.integral, 100 / self.Ki), -100 / self.Ki)
        print(f"raw pwm {self.Kp * self.error + self.Ki * self.integral}")
        output = max(min(self.Kp * self.error + self.Ki * self.integral, 100), 0) # + self.Kd * derivative
        return output

class Motor:
    def __init__(self, in1:int=5, in2:int=6, name="motor1"):
        self.in1 = in1
        self.in2 = in2
        self.name = name
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        self.pwm_in1 = GPIO.PWM(self.in1, 1000) # pwm frequency 2kHz on pin in1
        self.pwm_in2 = GPIO.PWM(self.in2, 1000)
        self.pwm_in1.start(0)
        self.pwm_in2.start(0)
        self.pid_controller = PIDController()
        

    def stop(self):
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(0)

    def rotate_forward(self, pwm_value=0):
        self.pwm_in1.ChangeDutyCycle(pwm_value)
        self.pwm_in2.ChangeDutyCycle(0)

    def rotate_backward(self, pwm_value):
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(pwm_value)
    
    def rotate(self, u_n, rpm_n):
        target = abs(u_n)
        pwm = self.pid_controller.compute(target, rpm_n)
        (self.rotate_backward if u_n < 0 else self.rotate_forward)(pwm)

    def destroy(self):
        self.pwm_in1.stop()
        self.pwm_in2.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")