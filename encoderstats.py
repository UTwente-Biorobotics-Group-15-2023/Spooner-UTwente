from biorobotics import Encoder
from pin_definitions import Pins
from ulab import numpy as np

class EncoderStats(object):
    def __init__(self, selected_motor, ticker_frequency):
        if selected_motor == 1:
            pin_1 = Pins.MOTOR_1_ENCODER_1
            pin_2 = Pins.MOTOR_1_ENCODER_2
        elif selected_motor == 2:
            pin_1 = Pins.MOTOR_2_ENCODER_1
            pin_2 = Pins.MOTOR_2_ENCODER_2

        self.motor = selected_motor
        self.encoder = Encoder(pin_1, pin_2)
        self.home_state_offset = 0
        self.step_to_rad = 360 / 8400 * (np.pi / 180)
        self.period = 1/ticker_frequency
        return

    # wrapper, wraps every 8400 counts
    def get_count(self):
        # we want to make sure that wrapping happens on the correct side of the home state:
        # > m1 should increase the counter when goning counter clockwise, instead it decreases it
        # > m2 works good
        if self.motor == 1:
            self.counter = - self.encoder.counter() % 8400
        if self.motor == 2:
            self.counter = self.encoder.counter() % 8400
        return self.counter

    def get_angle(self):
        self.count = self.get_count()
        self.angle_radians = self.count * self.step_to_rad
        return self.angle_radians + self.home_state_offset

    # !! Please input only positive angles !! (e.g. map -pi/2 to 3pi/2)
    def set_angle(self, home_angle_rad):
        # no matter what we read rn, we are currently at home_angle_rad
        # we wanna set our counter to the corresponding value
        if self.motor == 1:
            self.encoder.set_counter( -round(home_angle_rad * 1/self.step_to_rad) )
        elif self.motor == 2:
            self.encoder.set_counter( round(home_angle_rad * 1/self.step_to_rad) )
        return
    
    # def set_angle(self, home_angle_rad):
    #     # no matter what we read rn, we are currently at home_angle
    #     # we wanna make sure we now read that 
    #     self.home_state_offset = home_angle_rad - self.get_angle()
    #     return
    
    def get_angular_velocity(self, angle_current, angle_previous):
        self.angular_velocity = (angle_current - angle_previous) / self.period
        return self.angular_velocity
