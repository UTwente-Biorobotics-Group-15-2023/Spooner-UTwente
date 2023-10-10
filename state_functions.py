from states import State
#from pyb import LED

class StateFunctions(object):

    def __init__(self, robot_state, sensor_state):
        #self.led = LED(1)
        self.robot_state = robot_state
        self.sensor_state = sensor_state
        self.callbacks = {
            State.CALLIBRATE: self.callibrate,
            State.HOME: self.home,
            State.HOLD: self.hold,
            State.MOVE: self.move,
            State.EMERGENCY_STOP: self.emergency_stop,
        }
        return

    def callibrate(self):
        # The below if statement makes sure we only execute entry action once
        # basically - checks if the states differ between this and previous update
        if self.robot_state.is_changed():
            ## Entry action
            print('Lets callibrate the EMG input!')
            print('Please remain still and relaxed for 10 seconds')
            print('When the LED turns off, you should contract your muscles as hard as you can')
            self.robot_state.set(State.CALLIBRATE)
        ## Main action

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.EMERGENCY_STOP)

        # TODO: if callibration done, move to HOME state

        return
    
    def home(self):
        if self.robot_state.is_changed():
            #self.led.on()
            ## Entry action
            self.robot_state.set(State.HOME)
        ## Main action

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.EMERGENCY_STOP)

        return
    
    def hold(self):
        if self.robot_state.is_changed():
            ## Entry action
            self.robot_state.set(State.HOLD)
        ## Main action

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.EMERGENCY_STOP)

        # TODO: add a state trasition to MOVE if CONTINUE signal is detected

        return
    
    def move(self):
        if self.robot_state.is_changed():
            ## Entry action
            self.robot_state.set(State.MOVE)
        ## Main action

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.EMERGENCY_STOP)

        # TODO: add a state trasition to HOLD if the HOLD signal is detected

        return
    
    def emergency_stop(self):
        if self.robot_state.is_changed():
            ## Entry action
            self.robot_state.set(State.EMERGENCY_STOP)
        ## Main action

        ## Exit guards

        # NO EXIT GUARDS as this is a final state - reboot robot to restart

        return
        