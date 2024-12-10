from robot.sensor import all_black, sensors, read_sensor, normalizeSensorValues, thresholdSensorValues

# Define the states
class States:
    INIT = "INIT"
    SERPENTINE = "SERPENTINE"
    STRAIGHTAWAY = "STRAIGHTAWAY"
    T_TURN = "T_TURN"
    FORK_AND_TURN = "FORK_AND_TURN"
    RETURN_TO_START = "RETURN_TO_START"

class StateMachine:
    def __init__(self):
        self.state = States.INIT
        self.readings = []

    def update_sensor_readings(self):
        # Read sensors and normalize them
        raw_values = [read_sensor(sensor) for sensor in sensors]
        normalized_values = normalizeSensorValues(raw_values)
        self.readings = thresholdSensorValues(normalized_values)

    def check_transition(self):
        # Transition logic based on state and sensor readings
        if self.state == States.INIT:
            if all_black(self.readings):  # Wait for all black to start
                return States.SERPENTINE

        elif self.state == States.SERPENTINE:
            if all_black(self.readings):  # Detect all black to transition
                return States.STRAIGHTAWAY

        elif self.state == States.STRAIGHTAWAY:
            if all_black(self.readings):  # Detect all black to transition
                return States.T_TURN

        elif self.state == States.T_TURN:
            if all_black(self.readings):  # Detect all black to transition
                return States.FORK_AND_TURN

        elif self.state == States.FORK_AND_TURN:
            if all_black(self.readings):  # Detect all black to transition
                return States.RETURN_TO_START

        elif self.state == States.RETURN_TO_START:
            if all_black(self.readings):  # Detect all black to restart
                return States.INIT

        return self.state  # Stay in the current state if no transition detected

    def handle_state(self):
        # Placeholder for state-specific logic (executed in `code.py`)
        print(f"Handling state: {self.state}")

    def run(self):
        self.update_sensor_readings()
        next_state = self.check_transition()
        if next_state != self.state:
            print(f"Transitioning from {self.state} to {next_state}")
            self.state = next_state
        self.handle_state()


