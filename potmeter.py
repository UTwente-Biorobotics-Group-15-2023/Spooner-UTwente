from biorobotics import AnalogIn
from pin_definitions import Pins


class PotMeter(object):

    def __init__(self):
        self.potmeter_1_value = AnalogIn(Pins.POTMETER_1).read()  # Read value, result ranges from 0.0 to 1.0
        self.potmeter_2_value = AnalogIn(Pins.POTMETER_2).read()
        return

    def value(self):
        self.potmeter_1_value = AnalogIn(Pins.POTMETER_1).read()
        self.potmeter_2_value = AnalogIn(Pins.POTMETER_2).read()
        return_value = [self.potmeter_1_value, self.potmeter_2_value]

        return return_value
