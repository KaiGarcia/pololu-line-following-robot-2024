import time
import digitalio
import pwmio
from analogio import AnalogOut
from board import A3, A6, A7, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13
from robot import motor
from robot import sensor

left_speed = 0
right_speed = 0
i = 0
# Function that makes robot follow a line
def lineFollow(base_speed = 45, time_increment = 0.1):
    global left_speed, right_speed, i
    sensor_positions = [-3, -2, -1, 1, 2, 3]  # Positions of the sensors
    base_speed_left = base_speed  # Base speed for the left motor
    base_speed_right = base_speed  # Base speed for the right motor

    readings = []

    # Read decay times from sensors
    for sense in sensor.sensors:
        decay_time = sensor.read_sensor(sense)
        readings.append(decay_time)
    # Normalize Readings
    normalized_readings = sensor.normalizeSensorValues(readings, white_val=0, black_val=2000)
    binary_readings = sensor.thresholdSensorValues(normalized_readings, threshold=0.85)
    # Calculate the line position
    line_position = sensor.calculate_line_position(binary_readings, sensor_positions)
    # Adjust motor speeds based on line position
    if line_position is not None:
        turn_correction = line_position * 10
        i = 0
        if turn_correction > 0:
            left_speed = max(0, (base_speed_left + turn_correction))
            right_speed = 0
        elif turn_correction < 0:
            left_speed = 0
            right_speed = max(0, (base_speed_left - turn_correction))
        else:
            left_speed = base_speed
            right_speed = base_speed
    else:
        # Stop if line lost
        i += 1
        if i > 10:
            left_speed = 0
            right_speed = 0
        else:
            left_speed = left_speed
            right_speed = right_speed
    
    start_PID_time = time.monotonic()
    while (time.monotonic() - start_PID_time) <= time_increment:
        motor.setMotorsDuty(int(max(-100, min(100, left_speed))*(2**15/100)), int(max(-100, min(100, right_speed))*(2**15 / 100)))

    print(f"Speed for Both Motors: right:{right_speed} left:{left_speed}")
    print(f"Raw Decay Times: {readings}")
    print(f"Binary Readings: {binary_readings}")
    print(f"Line Position: {line_position}")
    # print(f"Motor Speeds: Left = {base_speed_left}, Right = {base_speed_right}")
    print(f"New Speed: Left = {left_speed}, Right = {right_speed}")



while True:
    lineFollow()