from ulab import numpy as np


class CompensationController(object):
    def __init__(self, ticker_frequency):
        self.velocity = 0
        self.angle_previous = 0
        self.dt = 1 / ticker_frequency
        self.c1 = 0.003
        self.c2 = -0.0015
        self.c3 = 6
        self.u_dry = 0
        self.u_viscous_and_backemf = 0
        self.u = 0

    def calculate_u(self, angle, angle_previous):
        self.velocity = (angle - angle_previous) / self.dt
        self.u_dry = self.c1 * np.tanh(self.c3 * self.velocity)
        self.u_viscous_and_backemf = self.c2 * self.velocity
        self.u = self.u_dry + self.u_viscous_and_backemf
        if self.u > 1:
            self.u = 1
        elif self.u < -1:
            self.u = -1
        return self.u
