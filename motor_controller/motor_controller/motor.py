from RPi.GPIO import GPIO


class Motor():
    def __init__(self, in1:int, in2:int):
        self.in1 = in1
        self.in2 = in2
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        self.pwm_in1 = GPIO.PWM(self.in1, 2000) # pwm frequency 2kHz on pin in1
        self.pwm_in2 = GPIO.PWM(self.in2, 2000)
        self.pwm_in1.start(0)
        self.pwm_in2.start(0)

    def rotate_clockwise(self, pwm_value:int):
        self.pwm_in1.ChangeDutyCycle(pwm_value)
        self.pwm_in2.ChangeDutyCycle(0)

    def rotate_anticlockwise(self, pwm_value:int):
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(pwm_value)

    def __del__(self):
        self.pwm_in1.stop()
        self.pwm_in2.stop()
        GPIO.cleanup()
        print("GPIO cleaned up")