from biorobotics import Ticker
from states import State
from sensor import SensorState
from state_functions import StateFunctions
from switch import BlueSwitch


class StateMachine(object):
    def __init__(self, ticker_frequency):
        self.robot_state = State()
        self.sensor_state = SensorState()
        self.state_functions = StateFunctions(self.robot_state, self.sensor_state)
        self.ticker = Ticker(0, ticker_frequency, self.run) #1 number on hardware timer, 2 frequency, 3 function you want to run
        

        return
    
    def run(self):
        self.sensor_state.update()
        #print(self.sensor_state.switch_value)
        self.state_functions.callbacks[self.robot_state.current]()
        return
    
    def start(self):
        print('robot starting')
        self.ticker.start()
        return
    
    def stop(self):
        print('robot shutting down')
        self.ticker.stop()
        return