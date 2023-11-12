from biorobotics import Ticker
from states import State
from sensor import SensorState
from state_functions import StateFunctions


class StateMachine(object):
    def __init__(self, ticker_frequency):
        self.robot_state = State()
        self.sensor_state = SensorState(ticker_frequency)
        self.state_functions = StateFunctions(self.robot_state, self.sensor_state, ticker_frequency)
        self.ticker = Ticker(0, ticker_frequency,
                             self.run)  # 1 number on hardware timer, 2 frequency, 3 function you want to run
        return

    # Function that is run by the Ticker at the ticker_frequency
    # updates the sensor state AND executes the current state function callback
    def run(self):
        self.sensor_state.update()
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
