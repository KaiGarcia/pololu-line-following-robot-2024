from sensor import all_black, sensors, read_sensor, normalizeSensorValues, thresholdSensorValues

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
        if all_black(self.readings):
            # Transition to the next state if we read all black
            if self.state == States.INIT:
                self.state = States.SERPENTINE
            elif self.state == States.SERPENTINE:
                self.state = States.STRAIGHTAWAY
            elif self.state == States.STRAIGHTAWAY:
                self.state = States.T_TURN
            elif self.state == States.T_TURN:
                self.state = States.FORK_AND_TURN
            elif self.state == States.FORK_AND_TURN:
                self.state = States.RETURN_TO_START
            elif self.state == States.RETURN_TO_START:
                self.state = States.INIT  # Loop back to INIT

    def handle_state(self):
        # Placeholder for handling the logic in each state
        if self.state == States.INIT:
            print("Waiting for all black to start...")
        elif self.state == States.SERPENTINE:
            print("Executing Serpentine logic...")
        elif self.state == States.STRAIGHTAWAY:
            print("Executing Straightaway logic...")
        elif self.state == States.T_TURN:
            print("Executing T-Turn logic...")
        elif self.state == States.FORK_AND_TURN:
            print("Executing Fork and Turn logic...")
        elif self.state == States.RETURN_TO_START:
            print("Returning to start...")

    def run(self):
        self.update_sensor_readings()
        self.check_transition()
        self.handle_state()

