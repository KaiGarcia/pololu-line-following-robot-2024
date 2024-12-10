import time
from robot import motor, sensor, state_machine, functions

# Initialize the state machine
state_machine_instance = state_machine.StateMachine()

# Main loop
if __name__ == "__main__":
    # Wait for button press at the start
    functions.button_press()

    while True:
        # Run the state machine to update and transition states
        state_machine_instance.run()

        # Get the current state from the state machine
        current_state = state_machine_instance.state

        if current_state == state_machine.States.INIT:
            print("State: INIT - Waiting for all black to start...")
            time.sleep(0.5)

        elif current_state == state_machine.States.SERPENTINE:
            print("State: SERPENTINE")

            functions.lineFollow()

        elif current_state == state_machine.States.STRAIGHTAWAY:
            print("State: STRAIGHTAWAY")
            functions.lineFollow()

        elif current_state == state_machine.States.T_TURN:
            print("State: T_TURN")
            functions.handleTurn(sensor.thresholdSensorValues(
                sensor.normalizeSensorValues(
                    [sensor.read_sensor(s) for s in sensor.sensors]
                )
            ))

        elif current_state == state_machine.States.FORK_AND_TURN:
            print("State: FORK AND TURN")
            functions.choosePath()

        elif current_state == state_machine.States.RETURN_TO_START:
            print("State: RETURN TO START")
            motor.turnDegrees(180)  # Perform a U-turn

        time.sleep(0.1)  # Add a small delay to avoid CPU overload


