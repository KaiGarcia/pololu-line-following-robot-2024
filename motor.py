import digitalio
import pwmio
import time
from board import D2, D3, D4, D5, D10, D11, D12, D13

# Setup for Motor A
ain1 = digitalio.DigitalInOut(D4)
ain1.direction = digitalio.Direction.OUTPUT
pwm_a = pwmio.PWMOut(D2, frequency=1000)

# Setup for Motor B
bin1 = digitalio.DigitalInOut(D5)
bin1.direction = digitalio.Direction.OUTPUT
pwm_b = pwmio.PWMOut(D3, frequency=1000)

# Change the direction pins

# Setup Encoders for Motor A
enc_a1 = digitalio.DigitalInOut(D10)
enc_a1.direction = digitalio.Direction.INPUT
enc_a2 = digitalio.DigitalInOut(D11)
enc_a2.direction = digitalio.Direction.INPUT

# Setup Encoder for Motor B
enc_b1 = digitalio.DigitalInOut(D12)
enc_b1.direction = digitalio.Direction.INPUT
enc_b2 = digitalio.DigitalInOut(D13)
enc_b2.direction = digitalio.Direction.INPUT



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

# Function that measures the speed of the encoders and outputs 
def encoder(time_step):
    start_time = time.monotonic()

    # Set the previous states for Motor A and Motor B
    last_state_A = False
    last_state_B = False

    # Set counts for pulses
    pulse_A = 0
    pulse_B = 0
    while (time.monotonic() - start_time) <= time_step:
        # Motor A - A XOR B
        current_state_A = (enc_a1.value and not enc_a2.value) or (not enc_a1.value and enc_a2.value)
        # Motor B - A XOR B
        current_state_B = (enc_b1.value and not enc_b2.value) or (not enc_b1.value and enc_b2.value)

        # Count when the current states of the encoder change to count the number of cycles
        if current_state_A != last_state_A:
            pulse_A += 1
            last_state_A = current_state_A
        if current_state_B != last_state_B:
            pulse_B += 1
            last_state_B = current_state_B
    
    # Convert pulse count into number of rotations
    pulse_per_rotation = 1440
    rotations_A = pulse_A / pulse_per_rotation
    rotations_B = pulse_B / pulse_per_rotation

    # Convert rotations into rpm
    rpm_A = rotations_A * 60 / time_step
    rpm_B = rotations_B * 60 / time_step
    
    return rpm_A, rpm_B

while True:
    print(encoder(0.5))
    #time.sleep(1)
