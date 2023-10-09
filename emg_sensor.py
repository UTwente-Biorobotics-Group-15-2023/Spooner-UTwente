from biorobotics import Ticker, AnalogIn, SerialPC
from biquad import  Biquad
import ulab as np


emgs = [AnalogIn('A0'), AnalogIn('A1'), AnalogIn('A2')]

# Initialize filters obtained from MATLAB's filterdesigner 
# 300Hz fs; 10Hz cut off Butterworth low pass
LP_100_10_1 = Biquad(-1.705552145544083852968242354108951985836,   0.743655195048865791385139800695469602942, 1.0, 2.0, 1.0)
gain_1 = 0.009525762376195455113925270040908799274


# 300Hz fs; 48-52 band stop Butterworth
LP_100_10_2 = Biquad(-0.960616192564186621716260106040863320231,  0.919547137907040124105151335243135690689, 1.0, -1.000877940003687793790732030174694955349, 1.0)
gain_2 = 0.959773568953520062052575667621567845345 

class EmgSensor(): 

    def __init__(self):
        self.emg_sensor_value = np.zeros(len(emgs)) # value of EMG [sensor 1, sensor 2, sensor 3]
        return
    
    def value(self): #return the current filtered value 
        
        for i in enumerate(emgs):
            self.emg_sensor_value[i] = gain_1 * LP_100_10_1.filter(emgs[i].read()) # returns filtered signal by applying filter function from Biquad class
            
        return_value = self.emg_sensor_value
        return return_value
