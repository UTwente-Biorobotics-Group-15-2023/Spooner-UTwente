from states import State

class StateFunctions(object):

    def __init__(self, robot_state, sensor_state):
        
        self.robot_state = robot_state
        self.sensor_state = sensor_state

        self.callbacks = {
            State.GREEN: self.green,
            State.YELLOW: self.yellow,
            State.RED: self.red}

        return

    def green(self):
        ## Entry action
        if self.robot_state.is_changed():
            self.robot_state.set(State.GREEN)

        ## Main action

    

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.YELLOW)

        return
    
    def yellow(self):
        
        ## Entry action
        if self.robot_state.is_changed():
            self.robot_state.set(State.YELLOW)

        ## Main action
    
        ## Exit guards        
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.RED)
        return
    
    def red(self):

        ## Entry action
        if self.robot_state.is_changed():
            self.robot_state.set(State.RED)

        ## Main action
           

        ## Exit guards        
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.GREEN)
        return
        