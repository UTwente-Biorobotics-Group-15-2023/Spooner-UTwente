#keeps track of sensor data
from switch import BlueSwitch
from emg_sensor import EmgSensor

class SensorState(object):

    def __init__(self):
        self.switch_value = 0
        self.blue_switch = BlueSwitch()

        self.emg_sensor = EmgSensor() #returns the filtered signal
        return
    
    def update(self):
        self.switch_value = self.blue_switch.value()
        self.emg_sensor = self.emg_sensor.value() # gives the current value of the emg sensor in vector form with [value emg1, value emg2, value emg3]
        return