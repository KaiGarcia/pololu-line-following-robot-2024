import time
import digitalio
import pwmio
from board import D2, D3, D4, D5, D6, D7, D8, D9, D10, D11

# Reflectance sensors
sensors = [
    digitalio.DigitalInOut(D6),
    digitalio.DigitalInOut(D7),
    digitalio.DigitalInOut(D8),
    digitalio.DigitalInOut(D9),
    digitalio.DigitalInOut(D10),
    digitalio.DigitalInOut(D11),
]

for sensor in sensors:
    sensor.direction = digitalio.Direction.INPUT

# Setup for Motor A
ain1 = digitalio.DigitalInOut(D4)
ain1.direction = digitalio.Direction.OUTPUT
pwm_a = pwmio.PWMOut(D2, frequency=1000)

# Setup for Motor B
bin1 = digitalio.DigitalInOut(D5)
bin1.direction = digitalio.Direction.OUTPUT
pwm_b = pwmio.PWMOut(D3, frequency=1000)

# Function to read a single sensor
def read_sensor(sensor):
    sensor.direction = digitalio.Direction.OUTPUT
    sensor.value = True
    time.sleep(0.00001)  # Charge for 10 μs
    sensor.direction = digitalio.Direction.INPUT
    start_time = time.monotonic()
    while sensor.value:
        pass
    end_time = time.monotonic()
    return (end_time - start_time) * 1_000_000  # Decay time in μs

# Normalize sensor values
def normalizeSensorValues(sensor_values, white_val=500, black_val=2000):
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
    if num_sensors == 0:
        return True
    else:
        return False

# Set motor speeds
def setMotorsDuty(dutyA, dutyB):
    motors = {'A': (ain1, pwm_a, dutyA), 'B': (bin1, pwm_b, dutyB)}
    for motor, (in1, pwm, dutyCycle) in motors.items():
        if dutyCycle > 0:  # Forward (CW)
            in1.value = True
            pwm.duty_cycle = int((dutyCycle / 100) * 65535)
        elif dutyCycle < 0:  # Backward (CCW)
            in1.value = False
            pwm.duty_cycle = int((abs(dutyCycle) / 100) * 65535)
        else:  # Stop
            in1.value = False
            pwm.duty_cycle = 0

def lineFollow(base_speed, time_increment):
    sensor_positions = [-3, -2, -1, 1, 2, 3]  # Positions of the sensors
    base_speed_left = base_speed  # Base speed for the left motor
    base_speed_right = base_speed  # Base speed for the right motor

    readings = []
    
    # Read decay times from sensors
    for sensor in sensors:
        decay_time = read_sensor(sensor)
        readings.append(decay_time)
    # Normalize Readings
    normalized_readings = normalizeSensorValues(readings, white_val=50000, black_val=80000)
    binary_readings = thresholdSensorValues(normalized_readings, threshold=0.7)
    # Calculate the line position
    line_position = calculate_line_position(binary_readings, sensor_positions)
    # Adjust motor speeds based on line position
    if line_position is not None:
        turn_correction = line_position * 5   # Scale to fix direction
        left_speed = base_speed_left - turn_correction
        right_speed = base_speed_right + turn_correction
        setMotorsDuty(max(0, min(100, left_speed)), max(0, min(100, right_speed)))
    else:
        # if centered keep running at the same speed
        setMotorsDuty(-base_speed_left, -base_speed_right)
    
    print(f"Speed for Both Motors: right:{right_speed} left:{left_speed}")
    print(f"Raw Decay Times: {readings}")
    print(f"Binary Readings: {binary_readings}")
    print(f"Line Position: {line_position}")
    print(f"Motor Speeds: Left = {base_speed_left}, Right = {base_speed_right}")
    print(f"New Speed: Left = {left_speed}, Right = {right_speed}")

    time.sleep(time_increment)  # Wait for time_increment seconds before making next adjustment


