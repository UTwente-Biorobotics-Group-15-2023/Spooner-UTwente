from machine import Pin
from biorobotics import PWM
from pin_definitions import Pins


class Motor(object):

    def __init__(self, frequency, motor=1):

        if motor == 1:
            self.pwm = PWM(Pins.MOTOR_1_PWM, frequency)
            self.direction = Pin(Pins.MOTOR_1_DIRECTION, Pin.OUT)

        else:
            self.pwm = PWM(Pins.MOTOR_2_PWM, frequency)
            self.direction = Pin(Pins.MOTOR_2_DIRECTION, Pin.OUT)

        return

    def write(self, pwm_value):
        """
        =INPUT=
            pwm_value - float
                Between -1 and 1
                Direction change is handled based on the sign
        
        """

        if pwm_value < 0:
            self.direction.value(1)  # set direction
        else:
            self.direction.value(0)

        self.pwm.write(abs(pwm_value))

        return
