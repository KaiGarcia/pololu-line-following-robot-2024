import time
import digitalio
import pwmio
from analogio import AnalogOut
from board import A3, A6, A7, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13

# Setup for Motor A
ain1 = digitalio.DigitalInOut(A6)
ain1.direction = digitalio.Direction.OUTPUT
pwm_a = pwmio.PWMOut(D6, frequency=1000)

# Setup for Motor B
bin1 = digitalio.DigitalInOut(A7)
bin1.direction = digitalio.Direction.OUTPUT
pwm_b = pwmio.PWMOut(D7, frequency=1000)

# Setup Encoders for Motor A
enc_a1 = digitalio.DigitalInOut(D2)
enc_a1.direction = digitalio.Direction.INPUT
enc_a2 = digitalio.DigitalInOut(D3)
enc_a2.direction = digitalio.Direction.INPUT

# Setup Encoder for Motor B
enc_b1 = digitalio.DigitalInOut(D4)
enc_b1.direction = digitalio.Direction.INPUT
enc_b2 = digitalio.DigitalInOut(D5)
enc_b2.direction = digitalio.Direction.INPUT

# Function that measures the speed of the encoders and outputs
last_state_A = False
last_state_B = False
fwd_A = 1
fwd_B = 1

def encoder(time_step):
    start_time = time.monotonic()
    # Set the previous states for Motor A and Motor B
    global last_state_A, last_state_B, fwd_A, fwd_B
    last_A_fwd = enc_a1.value
    last_B_fwd = enc_b1.value

    # Set counts for pulses
    pulse_A = 0
    pulse_B = 0
    while (time.monotonic() - start_time) <= time_step:
        # Motor A - A XOR B
        current_state_A = (enc_a1.value and not enc_a2.value) or (not enc_a1.value and enc_a2.value)
        A_fwd = enc_a1.value
        # Motor B - A XOR B
        current_state_B = (enc_b1.value and not enc_b2.value) or (not enc_b1.value and enc_b2.value)

        # Count when the current states of the encoder change to count the number of cycles
        if current_state_A != last_state_A:
            pulse_A += 1
            last_state_A = current_state_A
            if last_A_fwd != enc_a1.value:
                if enc_a2.value != enc_a1.value:
                    fwd_A = 1
                else:
                    fwd_A = -1
        last_A_fwd = enc_a1.value
        if current_state_B != last_state_B:
            pulse_B += 1
            last_state_B = current_state_B
            if last_B_fwd != enc_b1.value:
                if enc_b2.value != enc_b1.value:
                    fwd_B = -1
                else:
                    fwd_B = 1
        last_B_fwd = enc_b1.value

    # Convert pulse count into number of rotations
    pulse_per_rotation = 1440
    rotations_A = pulse_A / pulse_per_rotation
    rotations_B = pulse_B / pulse_per_rotation

    # Convert rotations into rpm
    rpm_A = fwd_A*rotations_A * 60 / time_step
    rpm_B = fwd_B*rotations_B * 60 / time_step

    return rpm_A, rpm_B

def calibrate_encoder(time_step):
    start_time = time.monotonic()
    # Set the previous states for Motor A and Motor B
    global last_state_A, last_state_B, fwd_A, fwd_B
    last_A_fwd = enc_a1.value
    last_B_fwd = enc_b1.value

    # Set counts for pulses
    pulse_A = 0
    pulse_B = 0
    while (time.monotonic() - start_time) <= time_step:
        # Motor A - A XOR B
        current_state_A = (enc_a1.value and not enc_a2.value) or (not enc_a1.value and enc_a2.value)
        A_fwd = enc_a1.value
        # Motor B - A XOR B
        current_state_B = (enc_b1.value and not enc_b2.value) or (not enc_b1.value and enc_b2.value)

        # Count when the current states of the encoder change to count the number of cycles
        if current_state_A != last_state_A:
            pulse_A += 1
            last_state_A = current_state_A
            if last_A_fwd != enc_a1.value:
                if enc_a2.value != enc_a1.value:
                    fwd_A = 1
                else:
                    fwd_A = -1
        last_A_fwd = enc_a1.value
        if current_state_B != last_state_B:
            pulse_B += 1
            last_state_B = current_state_B
            if last_B_fwd != enc_b1.value:
                if enc_b2.value != enc_b1.value:
                    fwd_B = -1
                else:
                    fwd_B = 1
        last_B_fwd = enc_b1.value

    # Convert pulse count into number of rotations
    print(pulse_A, pulse_B)


# Set up error terms for PID tuning
previous_error_A = 0
integral_error_A = 0
previous_error_B = 0
integral_error_B = 0
measureStart = time.monotonic()
A = 0
B = 1

# Perform PID tuning
def pid(mot, setpoint, current):
    global A, B, previous_error_A, integral_error_A, previous_error_B, integral_error_B, measureStart

    if mot == A:
        # Proportional, Derivative, and Integral Gains
        kp = 0.6
        kd = 0.2
        ki = 0.05
        feedforward = 5

        # Calculate the error
        error = setpoint - current

        # Calculate the time step
        dt = time.monotonic() - measureStart
        if dt == 0:  # Prevent division by zero
            dt = 0.1  # Use a small default value

        # Proportional term
        p = kp * error

        # Derivative term
        d = kd * (error - previous_error_A) / dt

        # Integral term
        integral_error_A += error * dt
        if integral_error_A >= 1200:
            integral_error_A = 1200

        i = ki * integral_error_A

        # Update previous values
        previous_error_A = error
        measureStart = time.monotonic()

        # Calculate the PID output
        update = p + d + i
        print(update)

        # Convert the output to a suitable duty cycle
        updateDuty = int(update * (2**15) / 500)

        if updateDuty >= (2**15):
            updateDuty = 2**15

        if updateDuty <= (-1*(2**15)):
            updateDuty = -2**15

        return updateDuty

    elif mot == B:
        # Proportional, Derivative, and Integral Gains
        kp = 0.6
        kd = 0.2
        ki = 0.05
        feedforward = 5

        # Calculate the error
        error = setpoint - current

        # Calculate the time step
        dt = time.monotonic() - measureStart
        if dt == 0:  # Prevent division by zero
            dt = 0.1  # Use a small default value

        # Proportional term
        p = kp * error

        # Derivative term
        d = kd * (error - previous_error_B) / dt
        # Integral term
        integral_error_B += error * dt
        if integral_error_B >= 1200:
            integral_error_B = 1200

        i = ki * integral_error_B

        # Update previous values
        previous_error_B = error
        measureStart = time.monotonic()

        # Calculate the PID output
        update = p + d + i
        print(update)

        # Convert the output to a suitable duty cycle
        updateDuty = int(update * (2**15) / 500)

        if updateDuty >= (2**15):
            updateDuty = 2**15

        if updateDuty <= (-1*(2**15)):
            updateDuty = -2**15

        return updateDuty
    else:
        return None

# Set motor speeds
def setMotorsDuty(dutyA, dutyB):
    motors = {'A': (ain1, pwm_a, dutyA), 'B': (bin1, pwm_b, dutyB)}
    for motor, (in1, pwm, dutyCycle) in motors.items():
        if dutyCycle < 0:  # Forward (CW)
            in1.value = True
            pwm.duty_cycle = int(abs(dutyCycle))
        elif dutyCycle > 0:  # Backward (CCW)
            in1.value = False
            pwm.duty_cycle = int(abs(dutyCycle))
        else:  # Stop
            in1.value = False
            pwm.duty_cycle = 0
    # print(f"Setting motors - Left Speed: {dutyA}, Right Speed: {dutyB}")

# Set Motor A and Motor B at a desired RPM value
def setMotorsPID(desired_rpm_A = 15, desired_rpm_B = 15, update_time = 0.1):
    # Measure the current RPMs for both motors
    current_rpm_A, current_rpm_B = encoder(update_time)

    # Print the measured RPMs for debugging
    # print(f"Measured RPM - Motor A: {current_rpm_A}, Motor B: {current_rpm_B}")

    # Calculate control signals only if RPM is non-zero
    control_signal_A = pid(A, desired_rpm_A, current_rpm_A)
    control_signal_B = pid(B, desired_rpm_B, current_rpm_B)

    # Apply control signals to the motors
    print(control_signal_A, control_signal_B)
    setMotorsDuty(control_signal_A, control_signal_B)

def drive_position(encoder_count_A, encoder_count_B):
    # Set the previous states for Motor A and Motor B
    global last_state_A, last_state_B, fwd_A, fwd_B
    last_A_fwd = enc_a1.value
    last_B_fwd = enc_b1.value

    # Set counts for pulses
    pulse_A = 0
    pulse_B = 0
    turn_count_A = 0
    turn_count_B = 0
    # Set for negative values to encoder_count
    if encoder_count_A < 0:
        set_duty_speed_A = -60
    else:
        set_duty_speed_A = 60

    if encoder_count_B < 0:
        set_duty_speed_B = -60
    else:
        set_duty_speed_B = 60

    setMotorsDuty(int(max(-100, min(100, set_duty_speed_A))*(2**15/100)), int(max(-100, min(100, set_duty_speed_B))*(2**15/100)))
    while ((abs(turn_count_A) < abs(encoder_count_A)) or (abs(turn_count_B) < abs(encoder_count_B))):
        current_state_A = (enc_a1.value and not enc_a2.value) or (not enc_a1.value and enc_a2.value)
        current_state_B = (enc_b1.value and not enc_b2.value) or (not enc_b1.value and enc_b2.value)

        if current_state_A != last_state_A:
            pulse_A += 1
            last_state_A = current_state_A
            if last_A_fwd != enc_a1.value:
                if enc_a2.value != enc_a1.value:
                    fwd_A = 1
                else:
                    fwd_A = -1
        last_A_fwd = enc_a1.value
        turn_count_A = pulse_A*fwd_A

        if current_state_B != last_state_B:
            pulse_B += 1
            last_state_B = current_state_B
            if last_B_fwd != enc_b1.value:
                if enc_b2.value != enc_b1.value:
                    fwd_B = 1
                else:
                    fwd_B = -1
        last_B_fwd = enc_b1.value
        turn_count_B = pulse_B*fwd_B

        if abs(turn_count_A) >= abs(encoder_count_A):
            set_duty_speed_A = 0
            setMotorsDuty(int(max(-100, min(100, set_duty_speed_A))*(2**15/100)), int(max(-100, min(100, set_duty_speed_B))*(2**15/100)))
        if abs(turn_count_B) >= abs(encoder_count_B):
            set_duty_speed_B = 0
            setMotorsDuty(int(max(-100, min(100, set_duty_speed_A))*(2**15/100)), int(max(-100, min(100, set_duty_speed_B))*(2**15/100)))

def inchForward(rotation = 0.2):
    c = int(1440 * rotation)
    drive_position(c, c)

def turnDegrees(angle):
    c = angle*8
    drive_position(c, -c)

def brake(brake_time = 0.05):
    delay_time = brake_time
    delay_start = time.monotonic()
    while (time.monotonic() - delay_start) <= delay_time:
        setMotorsDuty(0, 0)