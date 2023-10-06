#keeps track of the current state and can update the state
class State(object):
    GREEN = 'green'
    YELLOW = 'yellow'
    RED = 'red'


    def __init__(self):
        self.previous = None
        self.current = self.RED
    
        return
    
    def set(self, state):
        self.previous = self.current
        self.current = state
        return
    
    def is_changed(self):
        return self.previous is not self.current #if previous is not the same as current it would return True so then something changed

