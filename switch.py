from pyb import Switch
from pyb import Pin


class BlueSwitch(object):

    def __init__(self):
        self.switch_value = 0
        self.switch = Switch()
        self.switch.callback(self.callback)
        return

    def callback(self):
        self.switch_value = 1
        return

    def value(self):
        return_value = self.switch_value
        self.switch_value = 0
        return return_value


class KillSwitchOne(object):

    def __init__(self):
        self.pin_button = Pin('D70', Pin.IN, Pin.PULL_UP)
        return

    def value(self):
        return self.pin_button.value()  # Yields 1 or 0


class KillSwitchTwo(object):

    def __init__(self):
        self.pin_button = Pin('D69', Pin.IN, Pin.PULL_UP)
        return

    def value(self):
        return self.pin_button.value()  # Yields 1 or 0
