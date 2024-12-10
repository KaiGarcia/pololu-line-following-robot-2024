# Functions for Course Plus Button
import time
import random
import digitalio
from board import D13
from robot import motor
from robot import sensor

# Setup for Button
button = digitalio.DigitalInOut(D13)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP

# Parameters
left_speed = 0
right_speed = 0
i = 0

# Function for Putton Press
def button_press(debounce_delay=0.5):
    while True:
        if not button.value:
            print("Button pressed!")
            time.sleep(debounce_delay)
            return

# Function that makes robot follow a line
def lineFollow(base_speed = 15, time_increment = 0.1):
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
        turn_correction = line_position * 20
        i = 0
        if turn_correction > 0:
            left_speed = max(0, (base_speed_left + turn_correction))
            right_speed = base_speed
        elif turn_correction < 0:
            left_speed = base_speed
            right_speed = max(0, (base_speed_left - turn_correction))
        else:
            left_speed = base_speed*3
            right_speed = base_speed*3
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
        motor.setMotorsDuty(int(max(-100, min(100, left_speed))*(2**15/100)), int(max(-100, min(100, right_speed))*(2**15 / 100)))

    print(f"Speed for Both Motors: right:{right_speed} left:{left_speed}")
    print(f"Raw Decay Times: {readings}")
    print(f"Binary Readings: {binary_readings}")
    print(f"Line Position: {line_position}")
    # print(f"Motor Speeds: Left = {base_speed_left}, Right = {base_speed_right}")
    print(f"New Speed: Left = {left_speed}, Right = {right_speed}")

# Function to check for specific turn patterns
def handleTurn(binary_readings):
    if binary_readings == [0, 0, 0, 1, 1, 1]:
        print("Executing right turn")
        motor.turnDegrees(90)  # Right turn of 90 degrees

    elif binary_readings == [1, 1, 1, 0, 0, 0]:
        print("Executing left turn")
        motor.turnDegrees(-90)  # Left turn of 90 degrees
    else:
        print("Line following")
        lineFollow()  # Default to line-following if no turn pattern is detected

# Function to choose a path
def choosePath():
    # Generate a random number between 1 and 3
    path_choice = random.randint(1, 3)

    # Map the random number to a path
    if path_choice == 1:
        print("Path 1 selected: Turn left.")
        motor.turnDegrees(-45)
        lineFollow()
    elif path_choice == 2:
        print("Path 2 selected: Go straight.")
        lineFollow()
    elif path_choice == 3:
        print("Path 3 selected: Turn right.")
        motor.turnDegrees(45)
        lineFollow()
