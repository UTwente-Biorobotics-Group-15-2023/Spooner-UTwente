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
        self.angle_conversion_constant = (np.pi / 180)
        return

    def get_angle(self):
        self.count = self.encoder.counter() % 8400
        self.angle = self.count * 360 / 8400
        self.angle_radians = self.angle * self.angle_conversion_constant
        return self.angle_radians

    def get_angular_velocity(self):

        return
