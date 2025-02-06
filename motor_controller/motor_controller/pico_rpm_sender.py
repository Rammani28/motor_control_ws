# LM393Tachometer
# Specs : 20 slits on encoder disk and rpm is calculated every second (1000 ms)

import rp2
from machine import Pin, Timer, UART

uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))  # UART setup

# PIO Assembly Code for Pulse Counting
@rp2.asm_pio(set_init=rp2.PIO.IN_LOW)
def pulse_counter():
    wrap_target()
    label("wait")
    wait(1, pin, 0)  # Wait for pin to go HIGH
    wait(0, pin, 0)  # Wait for pin to go LOW (falling edge detection)
    jmp(x_dec, "wait")  # Decrement X register (counts pulses)
    wrap()

class Tachometer:
    def __init__(self, data_pin_num, sm_id, slits=20, calc_interval=1000): 
        self.pin = Pin(data_pin_num, Pin.IN)
        self.sm_id = sm_id
        self.slits = slits
        self.calc_interval = calc_interval
        self.rpm = 0
        
        # Initialize PIO State Machine for pulse counting
        self.sm = rp2.StateMachine(self.sm_id, pulse_counter, freq=1_000_000, in_base=self.pin)
        self.sm.exec("mov(x, invert(null))")  # Reset counter
        self.sm.active(1)  # Start PIO state machine

    def calc_rpm(self):
        self.sm.exec("mov(isr, x)")  
        self.sm.exec("push()")  
        pulse_count = 0xFFFFFFFF - self.sm.get()  
        self.sm.exec("mov(x, invert(null))")  
        #self.rpm = (pulse_count / self.slits) * (60 / (self.calc_interval / 1000))
        self.rpm = (pulse_count * 3)

    def get_rpm(self):
        return self.rpm

    def stop(self):
        self.sm.active(0)  # Stop PIO state machine


# Create instances for 3 tachometers
taco1 = Tachometer(16, 0)  # GP16
taco2 = Tachometer(17, 1)  # GP17
taco3 = Tachometer(18, 2)  # GP18

# Timer callback to update RPM and send via UART
def update_rpm(timer):
    taco1.calc_rpm()
    taco2.calc_rpm()
    taco3.calc_rpm()
    #rpm_data = f"rpm1:{taco1.get_rpm()}, rpm2:{taco2.get_rpm()}, rpm3:{taco3.get_rpm()}\n"
    
      # Send all RPMs at once
    #print(rpm_data)

def send_rpm(timer):
    rpm_data = f"{taco1.get_rpm()},{taco2.get_rpm()},{taco3.get_rpm()}\n"
    uart.write(rpm_data)
    print(f"sent{rpm_data}")

# Timer for continuous RPM calculation and transmission
rpm_timer = Timer()
uart_timer = Timer()

rpm_timer.init(period=1000, mode=Timer.PERIODIC, callback=update_rpm)
uart_timer.init(period=200, mode=Timer.PERIODIC, callback=send_rpm)

print("Code running...")

