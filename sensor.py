#keeps track of sensor data
from switch import BlueSwitch

class SensorState(object):

    def __init__(self):
        self.switch_value = 0
        self.blue_switch = BlueSwitch()
        return
    
    def update(self):
        self.switch_value = self.blue_switch.value()
        return