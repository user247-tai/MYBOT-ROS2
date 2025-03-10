from enum import Enum
class AutonomousMode(Enum):
    GOAL_NAVIGATE = 1
    PATROL = 2
    SHUTTLE = 3

class OperationStateMachine:
    def __init__(self):
        # Initial state is GOAL_NAVIGATE
        self.state = AutonomousMode.PATROL

    # Method to update state based on isPatrol and isShuttle
    def update_state(self, mode):
        if mode == 'PATROL' :
            self.state = AutonomousMode.PATROL
        elif mode == 'SHUTTLE':
            self.state = AutonomousMode.SHUTTLE
        elif mode == 'GOAL_NAVIGATE':
            self.state = AutonomousMode.GOAL_NAVIGATE

    # Method to get the current state
    def get_state(self):
        return self.state 
    def get_state_string(self):
        if self.state == AutonomousMode.PATROL:
            return 'PATROL'
        elif self.state == AutonomousMode.SHUTTLE:
            return 'SHUTTLE'
        else:
            return 'GOAL_NAVIGATE'