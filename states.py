# keeps track of the current state and can update the state
class State(object):
    # The state definitions are directly accessible from the Class without creating a specific instance (object) of it
    CALLIBRATE = 'callibrate'
    HOME = 'home'
    HOLD = 'hold'
    MOVE = 'move'
    EMERGENCY_STOP = 'emergency_stop'

    def __init__(self):
        self.previous = None
        self.current = self.CALLIBRATE
        return

    def set(self, state):
        self.previous = self.current
        self.current = state
        return

    def is_changed(self):
        # if previous STATE is not same as current, state has changed, so return boolean "True"
        return self.previous is not self.current
