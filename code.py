import time
from board import LED_R, LED_B, LED_G
from robot import motor, sensor, state_machine, functions
import digitalio

# Setup on-board LED
led_red = digitalio.DigitalInOut(LED_R)
led_red.direction = digitalio.Direction.OUTPUT

led_green = digitalio.DigitalInOut(LED_G)
led_green.direction = digitalio.Direction.OUTPUT

led_blue = digitalio.DigitalInOut(LED_B)
led_blue.direction = digitalio.Direction.OUTPUT

# Initialize all LEDs to OFF
led_red.value = 1
led_blue.value = 1
led_green.value = 1

# Set the color of the LEDs
def setRGB(red, green, blue):
    led_red.value = red
    led_green.value = green
    led_blue.value = blue

# Initialize the state machine
state_machine_instance = state_machine.StateMachine()

# Initialize the current state
current_state = state_machine.States.RETURN_TO_START

# Initialize the last sensor state
last_sensor_turn = [1, 1, 1, 1, 1, 1]
start_time = time.monotonic()
# Main loop
if __name__ == "__main__":
    # Wait for button press at the start
    functions.button_press()

    while True:
        # Run the state machine to update and transition states
        state_machine_instance.run()
        
        # Check if state changes and perform some initial conditions
        if (current_state != state_machine_instance.state):
            # Pause before beginning the next action
            motor.brake()
            
            if state_machine_instance.state == state_machine.States.INIT: 
                setRGB(0, 1, 1)               # RED FOR INIT
            if state_machine_instance.state == state_machine.States.SERPENTINE: 
                setRGB(0, 0, 1)         # YELLOW FOR SERPENTINE
                # Move slightly forward to dodge the black line
                motor.inchForward()
            if state_machine_instance.state == state_machine.States.STRAIGHTAWAY: 
                setRGB(1, 0, 1)       # GREEN FOR STRAIGHTAWAY
                # Move slightly forward to dodge the black line
                motor.inchForward()
            if state_machine_instance.state == state_machine.States.T_TURN: 
                setRGB(1, 1, 0)             # BLUE FOR T_TURN
                if state_machine_instance.reverse:
                    motor.inchForward()
                    if chosen_path == 1:
                        motor.brake()
                        motor.turnDegrees(-45)
                        motor.brake()
                    elif chosen_path == 2:
                        motor.inchForward()
                    elif chosen_path == 3:
                        motor.brake()
                        motor.turnDegrees(45)
                        motor.brake()
                    motor.inchForward()
                else:
                    # Move slightly forward to dodge the black line
                    motor.inchForward()
            if state_machine_instance.state == state_machine.States.FORK_AND_TURN: 
                setRGB(0, 1, 0)      # PURPLE FOR FORK_AND_TURN
                if state_machine_instance.reverse:
                    motor.inchForward(0.4)
                else:
                    # Pick a path
                    chosen_path = functions.choosePath()
            if state_machine_instance.state == state_machine.States.RETURN_TO_START: setRGB(0, 0, 0)    # WHITE FOR RETURN_TO_START
            
        
        # Get the current state from the state machine
        current_state = state_machine_instance.state

        if current_state == state_machine.States.INIT:
            print("State: INIT - Waiting for all black to start...")
            time.sleep(0.2)

        elif current_state == state_machine.States.SERPENTINE:
            print("State: SERPENTINE")
            functions.lineFollow(base_speed=42, sensor_weights = [-5, -3, -1, 1, 3, 5])

        elif current_state == state_machine.States.STRAIGHTAWAY:
            print("State: STRAIGHTAWAY")
            functions.lineFollow(base_speed=80, time_increment = 0.01, correction = 3)

        elif current_state == state_machine.States.T_TURN:
            print("State: T_TURN")
            current_sensor_turn = sensor.thresholdSensorValues(sensor.normalizeSensorValues(sensor.read_sensor_array(),white_val=0, black_val=1900), threshold = 0.65)
            functions.handleTurn(current_sensor_turn, last_sensor_turn)
            last_sensor_turn = current_sensor_turn

        elif current_state == state_machine.States.FORK_AND_TURN:
            print("State: FORK AND TURN")
            functions.lineFollow(base_speed=38, sensor_weights = [-5, -3, -1, 1, 3, 5])

        elif current_state == state_machine.States.RETURN_TO_START:
            print("State: RETURN TO START")
            motor.turnDegrees(180)  # Perform a U-turn
            motor.brake(0.2)


