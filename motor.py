import pwmio
import digitalio
import time
from board import A6, A7, D2, D3, D10, D11, D12, D13

# Setup for Motor A
ain1 = digitalio.DigitalInOut(A6)
pwm_a = pwmio.PWMOut(D2, frequency=1000)

# Setup for Motor B
bin1 = digitalio.DigitalInOut(A7)
pwm_b = pwmio.PWMOut(D3, frequency=1000)

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

# Set up error terms for PID tuning
previous_error = 0
integral_error = 0
measureStart = time.monotonic()

# Perform PID tuning
def pid(setpoint, current):
    global previous_error, integral_error, measureStart

    # Proportional, Derivative, and Integral Gains
    kp = 0.12
    kd = 0.05
    ki = 0.5
    feedforward = 40

    # Calculate the error
    error = setpoint - current

    # Calculate the time step
    dt = time.monotonic() - measureStart
    if dt == 0:  # Prevent division by zero
        dt = 1e-6  # Use a small default value

    # Proportional term
    p = kp * error

    # Derivative term
    d = kd * (error - previous_error) / dt

    # Integral term
    integral_error += error * dt
    if integral_error >= 1200:
        integral_error = 1200
    i = ki * integral_error

    # Update previous values
    previous_error = error
    measureStart = time.monotonic()

    # Calculate the PID output
    update = p + d + i

    # Convert the output to a suitable duty cycle
    updateDuty = int(update * (2**15) / 300)

    return updateDuty

# Function that sets the speed for each Motor independently
def setMotorsDuty(dutyA, dutyB):
    motors = {'A': (ain1, pwm_a, dutyA), 'B': (bin1, pwm_b, dutyB)}
    
    for motor, (in1, pwm, dutyCycle) in motors.items():
        # Set direction and duty cycle
        if dutyCycle > 0:  # Forward (CW)
            in1.value = 65535
            pwm.duty_cycle = int((dutyCycle / 100) * 65535)
        elif dutyCycle < 0:  # Backward (CCW)
            in1.value = 0
            pwm.duty_cycle = int((abs(dutyCycle) / 100) * 65535)
        else:  # Stop
            in1.value = 0
            pwm.duty_cycle = 0  # Stop motor


def setMotors(speedA, speedB):
    # Measure the current rpm of Motor A and Motor B
    calcTime = 0.1 # Time to make each PID update
    rpmA, rpmB = encoder(calcTime)

    motors = {'A': (ain1, pwm_a, rpmA, speedA), 'B': (bin1, pwm_b, rpmB, speedB)}

    for motor, (in1, pwm, rpm, speed) in motors.items():
        # Take PID measurement 
        x = pid(speed, rpm)
        if x < 0:
            in1.value = 0
            pwm.duty_cycle = abs(x)
        else:
            in1.value = 65535
            pwm.duty_cycle = abs(x)

# Set Motor A and Motor B at a desired RPM value
def setMotorsPID(desired_rpm_A = 25, desired_rpm_B = 25, update_time = 0.1):
    # Measure the current RPMs for both motors
    current_rpm_A, current_rpm_B = encoder(update_time)

    # Print the measured RPMs for debugging
    print(f"Measured RPM - Motor A: {current_rpm_A}, Motor B: {current_rpm_B}")

    # Calculate control signals only if RPM is non-zero
    control_signal_A = pid(desired_rpm_A, current_rpm_A)
    control_signal_B = pid(desired_rpm_B, current_rpm_B) 

    # Apply control signals to the motors
    setMotorsDuty(control_signal_A, control_signal_B)
