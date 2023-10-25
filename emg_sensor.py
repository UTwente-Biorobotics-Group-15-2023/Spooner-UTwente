from biorobotics import AnalogIn, SerialPC
from biquad import  Biquad
import ulab as np

# Initialize filters obtained from MATLAB's filterdesigner
# 300Hz fs; 10Hz cut off Butterworth low pass
LP_100_10_1 = Biquad(-1.705552145544083852968242354108951985836,   0.743655195048865791385139800695469602942, 1.0, 2.0, 1.0)
gain_1 = 0.009525762376195455113925270040908799274
# 300Hz fs; 48-52 band stop Butterworth
LP_100_10_2 = Biquad(-0.960616192564186621716260106040863320231,  0.919547137907040124105151335243135690689, 1.0, -1.000877940003687793790732030174694955349, 1.0)
gain_2 = 0.959773568953520062052575667621567845345
# 300Hz fs; 2Hz - 7Hz cut off Butterworth band pass
BP_filter = Biquad(-1.894566421316362658799903329054359346628, 0.900404044297840822075329469953430816531, 1, 0, -1)
BP_gain = 0.049797977851079575084547457208827836439

emgs = [AnalogIn('A0'), AnalogIn('A1'), AnalogIn('A2')]
pc = SerialPC(3)

class EmgSensor(object): 

    def __init__(self):
        self.emg_sensor_value = [0, 0, 0] # initial value of EMG [sensor 1, sensor 2, sensor 3]
        return
    
    def filtered_emg(self): #return the current filtered value and takes the absolute value 
        
        # TODO: revamp this code - we cannot use one filter instance for all three signals
        # for i, emg in enumerate(emgs):
        #     self.emg_sensor_value[i] = abs(BP_gain * BP_filter.filter(emgs[i].read())) # returns filtered signal by applying filter function from Biquad class and takes the absolute value
        #     pc.set(i, self.emg_sensor_value[i])

        # TODO: the below code is for filter testing on the 0th EMG analog input only
        self.emg_sensor_value[0] = abs(BP_gain * BP_filter.filter(emgs[0].read())) # returns filtered signal by applying filter function from Biquad class and takes the absolute value
        pc.set(0, emgs[0].read())           # Raw EMG signal at Ch0
        pc.set(1, self.emg_sensor_value[0]) # Filtered EMG signal at Ch1
        pc.set(2, 0)                        # Ch2 set to 0 for now
        
        pc.send()
        # return_value = self.emg_sensor_value
        return 0    # TODO: the ouput is set to 0 for now, this should be updated later on
    
    # def moving_av(self): # calculate the moving average to be finished 
        
    #     filter_data= self.filtered_emg()
    #     return_value = self.emg_sensor_value
    #     return return_value 