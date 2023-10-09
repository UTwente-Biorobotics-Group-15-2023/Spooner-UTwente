from biorobotics import Ticker, AnalogIn, SerialPC

print('Starting EMG script...')


class biquad:
    def __init__(self, b0, b1, b2, a1, a2):
        self.b0 = b0
        self.b1 = b1
        self.b2 = b2
        self.a1 = a1
        self.a2 = a2
        self.w1 = 0.0
        self.w2 = 0.0

    def filter(self, x):
        y = float(self.b0 * x + self.w1)
        self.w1 = float(self.b1 * x - self.a1 * y + self.w2)
        self.w2 = float(self.b2 * x - self.a2 * y)
        return y


# Initialize filters obtained from MATLAB's filterdesigner
# 300Hz fs; 2Hz - 7Hz cut off Butterworth pand pass
# Left bicep
LBF = biquad(1, 0, -1, -1.894566421316362658799903329054359346628, 0.900404044297840822075329469953430816531)
gain_1 = 0.049797977851079575084547457208827836439
# Right bicep
RBF = biquad(1, 0, -1, -1.894566421316362658799903329054359346628, 0.900404044297840822075329469953430816531)
gain_2 = 0.049797977851079575084547457208827836439

pc = SerialPC(3)

emgs = [AnalogIn('A0'), AnalogIn('A1')]

def loop():
    # for i, emg in enumerate(emgs), cfilter in enumerate(filters), gain in enumerate(gains):
        # pc.set(i, gain * cfilter.filter(emg.read()))

    pc.set(0, gain_1 * RBF.filter(AnalogIn('A0').read()))
    pc.set(1, gain_2 * LBF.filter(AnalogIn('A1').read()))
    pc.send()


ticker = Ticker(0, 300, loop, enable_gc=True)

ticker.start()