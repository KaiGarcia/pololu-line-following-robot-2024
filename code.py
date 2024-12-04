import time
import digitalio
import pwmio
from analogio import AnalogOut
from board import A6, A7, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13

# Reflectance sensors
sensors = [
    digitalio.DigitalInOut(D8),
    digitalio.DigitalInOut(D9),
    digitalio.DigitalInOut(D10),
    digitalio.DigitalInOut(D11),
    digitalio.DigitalInOut(D12),
    digitalio.DigitalInOut(D13),
]

for sensor in sensors:
    sensor.direction = digitalio.Direction.INPUT

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

# Function to read a single sensor
def read_sensor(sensor):
    sensor.direction = digitalio.Direction.OUTPUT
    sensor.value = True
    time.sleep(0.00002)  # Charge for 20 μs
    sensor.direction = digitalio.Direction.INPUT
    start_time = time.monotonic()
    while sensor.value:
        pass
    end_time = time.monotonic()
    return (end_time - start_time) * 1_000_000  # Decay time in μs

# Normalize sensor values
def normalizeSensorValues(sensor_values, white_val=50, black_val=2000):
    normalized_values = []
    for val in sensor_values:
        normalized_value = (val - white_val) / (black_val - white_val)
        normalized_value = max(0.0, min(1.0, normalized_value))  # Clamp to [0, 1]
        normalized_values.append(normalized_value)
    return normalized_values

# Normalize Values to 0 or 1
def thresholdSensorValues(normalized_values, threshold=0.7):
    return [1 if val > threshold else 0 for val in normalized_values]

# Calculate the weighted line position
def calculate_line_position(binary_readings, sensor_positions):
    num = 0
    den = 0
    for reading, position in zip(binary_readings, sensor_positions):
        num += reading * position
        den += reading
    if den == 0:  # Avoid division by zero
        return None
    return num / den

# Determine if all sensors over white
def all_white(binary_readings):
    num_sensors = 0
    # Check each sensor reading
    for reading in binary_readings:
        num_sensors += reading

    # Output true if all sensors detect white
    if num_sensors == 0:
        return True
    else:
        return False

# Determine if all sensors over black
def all_black(binary_readings):
    num_sensors = 0
    # Check each sensor reading
    for reading in binary_readings:
        num_sensors += reading

    # Output true if all sensors detect white
    if num_sensors == 6:
        return True
    else:
        return False

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

# Set Motor A and Motor B at a desired RPM value
def setMotorsPID(desired_rpm_A = 15, desired_rpm_B = 15, update_time = 0.1):
    # Measure the current RPMs for both motors
    current_rpm_A, current_rpm_B = encoder(update_time)

    # Print the measured RPMs for debugging
    print(f"Measured RPM - Motor A: {current_rpm_A}, Motor B: {current_rpm_B}")

    # Calculate control signals only if RPM is non-zero
    control_signal_A = pid(A, desired_rpm_A, current_rpm_A)
    control_signal_B = pid(B, desired_rpm_B, current_rpm_B)

    # Apply control signals to the motors
    print(control_signal_A, control_signal_B)
    setMotorsDuty(control_signal_A, control_signal_B)
left_speed = 0
right_speed = 0
i = 0
# Function that makes robot follow a line
def lineFollow(base_speed = 55, time_increment = 0.1):
    global left_speed, right_speed, i
    sensor_positions = [-3, -2, -1, 1, 2, 3]  # Positions of the sensors
    base_speed_left = base_speed  # Base speed for the left motor
    base_speed_right = base_speed  # Base speed for the right motor

    readings = []

    # Read decay times from sensors
    for sensor in sensors:
        decay_time = read_sensor(sensor)
        readings.append(decay_time)
    # Normalize Readings
    normalized_readings = normalizeSensorValues(readings, white_val=0, black_val=3000)
    binary_readings = thresholdSensorValues(normalized_readings, threshold=0.9)
    # Calculate the line position
    line_position = calculate_line_position(binary_readings, sensor_positions)
    # Adjust motor speeds based on line position
    if line_position is not None:
        turn_correction = line_position * 30
        i = 0
        if turn_correction > 0:
            left_speed = turn_correction
            right_speed = 0
        elif turn_correction < 0:
            left_speed = 0
            right_speed = -turn_correction
        else:
            left_speed = base_speed
            right_speed = base_speed
    else:
        # Stop if line lost
        i += 1
        if i > 20:
            left_speed = 0
            right_speed = 0
        else:
            left_speed = left_speed
            right_speed = right_speed
    
    start_PID_time = time.monotonic()
    while (time.monotonic() - start_PID_time) <= time_increment:
        setMotorsDuty(int(max(-100, min(100, left_speed))*(2**15/100)), int(max(-100, min(100, right_speed))*(2**15 / 100)))

    print(f"Speed for Both Motors: right:{right_speed} left:{left_speed}")
    print(f"Raw Decay Times: {readings}")
    print(f"Binary Readings: {binary_readings}")
    print(f"Line Position: {line_position}")
    print(f"Motor Speeds: Left = {base_speed_left}, Right = {base_speed_right}")
    print(f"New Speed: Left = {left_speed}, Right = {right_speed}")

while True:
    lineFollow()
