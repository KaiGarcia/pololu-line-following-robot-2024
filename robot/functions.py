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

turned_time = time.monotonic()

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
def lineFollow(base_speed = 30, time_increment = 0.06, correction = 9, sensor_weights = [-3, -2, -1, 1, 2, 3] ):
    global left_speed, right_speed, i
    sensor_positions = sensor_weights # Positions of the sensors
    base_speed_left = base_speed  # Base speed for the left motor
    base_speed_right = base_speed  # Base speed for the right motor

    readings = sensor.read_sensor_array()
    normalized_readings = sensor.normalizeSensorValues(readings, white_val=0, black_val=1900)
    binary_readings = sensor.thresholdSensorValues(normalized_readings, threshold=0.7)

    # Calculate the line position
    line_position = sensor.calculate_line_position(binary_readings, sensor_positions)

    # Adjust motor speeds based on line position
    if line_position is not None:
        turn_correction = line_position * correction
        i = 0
        if turn_correction > 0:
            left_speed = max(0, (base_speed_left + turn_correction))
            right_speed = max(0, (base_speed_left - turn_correction))
        elif turn_correction < 0:
            left_speed = max(0, (base_speed_left + turn_correction))
            right_speed = max(0, (base_speed_left - turn_correction))
        else:
            left_speed = base_speed*1.2
            right_speed = base_speed*1.2
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
def handleTurn(binary_readings, last_normalized_readings):
    global turned_time
    # Calculate sums for left and right groups
    left_sum = sum(last_normalized_readings[:3])   # Sum of first three sensors
    right_sum = sum(last_normalized_readings[3:])  # Sum of last three sensors
    
    min_buffer_time = 1.5
    
    if (sensor.all_white(binary_readings) == True) and ((time.monotonic() - turned_time)>= min_buffer_time):
        print(sensor.normalizeSensorValues(sensor.read_sensor_array(),white_val=0, black_val=1900))
        print(binary_readings)
        lineFollow(base_speed = 35, time_increment = 0.03, correction = 3)
        bin2 = sensor.thresholdSensorValues(sensor.normalizeSensorValues(sensor.read_sensor_array(),white_val=0, black_val=1900), threshold = 0.6)
        print(bin2)
        if (sensor.all_white(bin2) == True):
            if right_sum > left_sum:
                print("Executing right turn")
                motor.turnDegrees(-90)  # Right turn of 90 degrees
                turned_time = time.monotonic()
                motor.brake()

            elif right_sum < left_sum:
                print("Executing left turn")
                motor.turnDegrees(90)  # Left turn of 90 degrees
                motor.brake()
                turned_time = time.monotonic()

            else:
                print("No clear turn pattern previously. Line following.")
                lineFollow(base_speed = 35, time_increment = 0.03, correction = 3)  # Default to line following
        # Default to line-following if not all sensors detect white
            
    else:
        print("Line following")
        lineFollow(base_speed = 35, time_increment = 0.03, correction = 3)
# Function to choose a path
def choosePath(path_choice = None):
    # Generate a random number between 1 and 3
    if path_choice is None:
        path_choice = random.randint(1, 3)

    # Map the random number to a path
    if path_choice == 1:
        print("Path 1 selected: Turn left.")
        motor.turnDegrees(45)
        motor.brake()
        # Move slightly forward to dodge the black line
        motor.inchForward()
    elif path_choice == 2:
        print("Path 2 selected: Go straight.")
        # Move slightly forward to dodge the black line
        motor.inchForward()
    elif path_choice == 3:
        print("Path 3 selected: Turn right.")
        motor.turnDegrees(-45)
        motor.brake()
        # Move slightly forward to dodge the black line
        motor.inchForward()
    
    return path_choice

