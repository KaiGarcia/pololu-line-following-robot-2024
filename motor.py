import digitalio
import pwmio
import time
from board import D2, D3, D4, D5

# Setup for Motor A
ain1 = digitalio.DigitalInOut(D4)
ain1.direction = digitalio.Direction.OUTPUT
pwm_a = pwmio.PWMOut(D2, frequency=1000)

# Setup for Motor B
bin1 = digitalio.DigitalInOut(D5)
bin1.direction = digitalio.Direction.OUTPUT
pwm_b = pwmio.PWMOut(D3, frequency=1000)

# Function that sets the speed for each Motor independently
def setMotors(dutyA, dutyB):
    motors = {'A': (ain1, pwm_a, dutyA), 'B': (bin1, pwm_b, dutyB)}
    
    for motor, (in1, pwm, dutyCycle) in motors.items():
        # Set direction and duty cycle
        if dutyCycle > 0:  # Forward (CW)
            in1.value = True
            pwm.duty_cycle = int((dutyCycle / 100) * 65535)
        elif dutyCycle < 0:  # Backward (CCW)
            in1.value = False
            pwm.duty_cycle = int((abs(dutyCycle) / 100) * 65535)
        else:  # Stop
            in1.value = False
            pwm.duty_cycle = 0  # Stop motor

while True:
    setMotors(30,30) 
    time.sleep(1)
