import RPi.GPIO as GPIO # https://pypi.org/project/RPi.GPIO/


class PIDController:
    def __init__(self, Kp=1.0, Ki=0.1, Kd=0.05, min_output=12, max_output=100):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.error  = 0
        # self.last_error = 0
        self.min_output = min_output
        self.max_output = max_output

    def compute(self, target_pwm, current_pwm):
        self.error = target_pwm - current_pwm
        self.integral += self.error
        # derivative = error - self.last_error
        self.last_error = self.error
        output = self.Kp * self.error + self.Ki * self.integral  # + self.Kd * derivative
        return min(output, 100)


class Motor:
    def __init__(self, in1:int=5, in2:int=6):
        self.in1 = in1
        self.in2 = in2
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        self.pwm_in1 = GPIO.PWM(self.in1, 2000) # pwm frequency 2kHz on pin in1
        self.pwm_in2 = GPIO.PWM(self.in2, 2000)
        self.pwm_in1.start(0)
        self.pwm_in2.start(0)
        self.direction = 1
        self.pid_controller = PIDController()

    def rotate_forward(self, pwm_value):
        self.pwm_in1.ChangeDutyCycle(pwm_value)
        self.pwm_in2.ChangeDutyCycle(0)

    def rotate_backward(self, pwm_value):
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(pwm_value)
    
    def rotate(self, curr_pwm, target_pwm):
        if int(target_pwm) > 1: 
            pwm = self.pid_controller.compute(abs(target_pwm), curr_pwm)
            if target_pwm<0:
                self.rotate_backward(abs(pwm))
            else:
                self.rotate_forward(pwm)

    def destroy(self):
        self.pwm_in1.stop()
        self.pwm_in2.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")