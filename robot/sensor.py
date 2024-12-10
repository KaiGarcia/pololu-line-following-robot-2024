import time
import digitalio
import pwmio
from analogio import AnalogOut
from board import A3, A6, A7, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13

# Reflectance sensors
sensors = [
    digitalio.DigitalInOut(D8),
    digitalio.DigitalInOut(D9),
    digitalio.DigitalInOut(D10),
    digitalio.DigitalInOut(D11),
    digitalio.DigitalInOut(D12),
    digitalio.DigitalInOut(A3),
]

for sensor in sensors:
    sensor.direction = digitalio.Direction.INPUT

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

def calibrate_sensor():
    sensor_positions = [-3, -2, -1, 1, 2, 3]  # Positions of the sensors
    sum = [0, 0, 0, 0, 0, 0]
    iterations = 0
    min_iterations = 25

    meas_time = 1.0
    meas_start = time.monotonic()

    while (((time.monotonic() - meas_start) <= meas_time) or (iterations < min_iterations)):
        n = 0
        for sensor in sensors:
            decay_time = read_sensor(sensor)
            sum[n] = sum[n] + decay_time
            n += 1
        iterations += 1

    readings = [reading / iterations for reading in sum]

    # Normalize Readings
    normalized_readings = normalizeSensorValues(readings, white_val=0, black_val=2000)
    binary_readings = thresholdSensorValues(normalized_readings, threshold=0.85)
    # Calculate the line position
    line_position = calculate_line_position(binary_readings, sensor_positions)

    print(f"Raw Decay Times: {readings}")
    print(f"Normalized Sensor Readings: {normalized_readings}")
    print(f"Binary Readings: {binary_readings}")
    print(f"Line Position: {line_position}")
    return binary_readings

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

    # Output true if all sensors detect black
    if num_sensors == 6:
        return True
    else:
        return False
