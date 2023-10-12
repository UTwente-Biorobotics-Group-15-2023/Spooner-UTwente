#keeps track of sensor data
from switch import BlueSwitch
from switch import KillSwitchOne
from switch import KillSwitchTwo
#from emg_sensor import EmgSensor

class SensorState(object):

    def __init__(self):
        self.switch_value = 0
        self.ks_one_value = 0  # switch 1 for finding the arm's home state
        self.ks_two_value = 0  # switch 2 for finding the arm's home state

        self.blue_switch = BlueSwitch()
        self.kill_switch_one = KillSwitchOne()
        self.kill_switch_two = KillSwitchTwo()
       
        #self.emg_sensor = EmgSensor() # returns the filtered signal
        return
    
    def update(self):
        self.switch_value = self.blue_switch.value()
        self.ks_one_value = self.kill_switch_one.value()
        self.ks_two_value = self.kill_switch_two.value()
        #self.emg_sensor = self.emg_sensor.value() # gives the current value of the emg sensor in vector form with [value emg1, value emg2, value emg3]
        return