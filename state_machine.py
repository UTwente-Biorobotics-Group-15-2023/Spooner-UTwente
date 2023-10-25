from biorobotics import Ticker
from states import State
from sensor import SensorState
from state_functions import StateFunctions
from switch import BlueSwitch
from biorobotics import SerialPC

class StateMachine(object):
    def __init__(self, ticker_frequency):
        self.serial_pc = SerialPC(3) # serial used for monotoring, passed to self.sensor_state to have just 1 serial where everything is plotted
        self.robot_state = State()
        self.sensor_state = SensorState(self.serial_pc)
        self.state_functions = StateFunctions(self.robot_state, self.sensor_state, ticker_frequency)
        self.ticker = Ticker(0, ticker_frequency, self.run) #1 number on hardware timer, 2 frequency, 3 function you want to run
        
        return
    
    # Function that is run by the Ticker at the ticker_frequency
    # updates the sensor state AND executes the current state function callback
    def run(self):
        self.sensor_state.update()       
        # print(self.sensor_state.switch_value)
        # get the current state value and translate it to the callback function to execute
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