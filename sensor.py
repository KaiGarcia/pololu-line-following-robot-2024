import time
import board
from digitalio import DigitalInOut, Direction

'''
THIS VERSION ONLY USES SENSOR 4
'''

# Define the pin connected to Sensor 4
sensor_4_pin = board.D4  # Adjust this pin according to your wiring

# Initialize Sensor 4 as a DigitalInOut object
sensor_4 = DigitalInOut(sensor_4_pin)

# Function to read a single sensor channel (Sensor 4)
def read_sensor(sensor):
    # Drive the pin high to charge it
    sensor.direction = Direction.OUTPUT
    sensor.value = True
    time.sleep(0.00001)  # Wait for at least 10 μs

    # Set the pin to input and measure decay time
    sensor.direction = Direction.INPUT
    start_time = time.monotonic()
    
    # Wait for the pin to go low
    while sensor.value:
        pass
    
    end_time = time.monotonic()
    
    # Calculate decay time in microseconds
    decay_time = (end_time - start_time) * 1_000_000
    return decay_time

# Main loop to read Sensor 4
while True:
    decay_time_sensor_4 = read_sensor(sensor_4)
    print(f"Sensor 4: {decay_time_sensor_4:.2f} μs")
    
    time.sleep(0.1)  # Delay between readings
