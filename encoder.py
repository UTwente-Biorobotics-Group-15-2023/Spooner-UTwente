from biorobotics import Encoder
from pin_definitions import Pins

class Encoder_own(object):

    def __init__(self):
        self.encoder_1 = Encoder(Pins.MOTOR_1_ENCODER_1, Pins.MOTOR_1_ENCODER_2)
        self.count = 0
        return
    
    def norm_count(self): 
        self.count = self.encoder_1.counter() % 8400
        return self.count
    def get_angle(self):

        return
    
    def get_angular_velocity(self):

        return

