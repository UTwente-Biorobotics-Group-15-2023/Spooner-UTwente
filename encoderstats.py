from biorobotics import Encoder
from pin_definitions import Pins
from ulab import numpy as np

class EncoderStats(object):
    def __init__(self, selected_motor = 1):
        if selected_motor == 1:
            pin_1 = Pins.MOTOR_1_ENCODER_1
            pin_2 = Pins.MOTOR_1_ENCODER_2
        elif selected_motor == 2:
            pin_1 = Pins.MOTOR_2_ENCODER_1
            pin_2 = Pins.MOTOR_2_ENCODER_2

        self.encoder = Encoder(pin_1, pin_2)
        self.home_state_offset = 0
        self.angle_conversion_constant = (np.pi / 180)
        return

    def get_angle(self):
        self.count = self.encoder.counter() % 8400
        self.angle = self.count * 360 / 8400
        self.angle_radians = self.angle * self.angle_conversion_constant
        return self.angle_radians + self.home_state_offset
    
    def set_home_angle(self, home_angle):
        # no matter what we read rn, we are currently at home_angle
        # we wanna make sure we now read that 
        self.home_state_offset = home_angle - self.get_angle()
        return

    def get_angular_velocity(self):

        return
