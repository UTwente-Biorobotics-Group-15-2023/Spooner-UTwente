
class Biquad:
    def __init__(self, a1, a2, b0, b1, b2):
        self.a1 = a1
        self.a2 = a2
        self.b0 = b0
        self.b1 = b1
        self.b2 = b2
        self.w1 = 0.0
        self.w2 = 0.0

    def filter(self, x):
        y = float(self.b0*x + self.w1)
        self.w1 = float(self.b1*x - self.a1*y + self.w2)
        self.w2 = float(self.b2*x - self.a2*y)
        return y
    
    
