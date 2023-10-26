from biorobotics import AnalogIn, SerialPC
from biquad import  Biquad

# Initialize filters obtained from MATLAB's filterdesigner
# 300Hz fs; 10Hz cut off Butterworth low pass
# LP_100_10_1 = Biquad(-1.705552145544083852968242354108951985836,   0.743655195048865791385139800695469602942, 1.0, 2.0, 1.0)
# gain_1 = 0.009525762376195455113925270040908799274
# # 300Hz fs; 48-52 band stop Butterworth
# LP_100_10_2 = Biquad(-0.960616192564186621716260106040863320231,  0.919547137907040124105151335243135690689, 1.0, -1.000877940003687793790732030174694955349, 1.0)
# gain_2 = 0.959773568953520062052575667621567845345
# # 300Hz fs; 2Hz - 7Hz cut off Butterworth band pass
# BP_filter = Biquad(-1.894566421316362658799903329054359346628, 0.900404044297840822075329469953430816531, 1, 0, -1)
# BP_gain = 0.049797977851079575084547457208827836439


# THE NEW FILTERS - ONLY LET THROUGHT THE GREAT STUFF!
# 300Hz fs; 10Hz cut off Butterworth high pass
LP_10_low =         Biquad(-1.4182,    0.5528,   0.7427,  -1.4855,    0.7427)
# 300Hz fs; 48-52 band stop Butterworth
LP_48_52_stop =     Biquad(-0.9556,    0.9253,   0.9626,  -0.9556,    0.9626)
# 300Hz fs; 98-102 band stop Butterworth
LP_98_102_stop =    Biquad( 0.9855,    0.9439,   0.9855,   0.9439,    0.9711)

emg0 = AnalogIn('A0')
emg1 = AnalogIn('A1')
emg2 = AnalogIn('A2')
#emgs = [AnalogIn('A0'), AnalogIn('A1'), AnalogIn('A2')]
pc = SerialPC(3)

class EmgSensor(object): 


    def __init__(self):
        self.sample_list = []
        self.normalization_factor = 0.045 # we will determine this during calibration step, stronger person -> larger factor
        self.window_size = 80
        self.emg_sensor_value = [0, 0, 0] # initial value of EMG [sensor 1, sensor 2, sensor 3]
        return
    

    # Combination of multiple filters - filters out the DC component, the 50HZ and the 100Hz peaks
    def prefilter(self, data):
        # Filtering 10Hz high pass - getting rid of DC components
        fdata = LP_10_low.filter(data)
        # Filtering out 50Hz and 100Hz interferance peaks
        fdata =  LP_48_52_stop.filter(fdata)
        fdata = LP_98_102_stop.filter(fdata)
        return fdata
    

    # Moving average filter - averages over a given window size
    def moving_average(self, data):

        # take care of the sample list
        if len(self.sample_list) < self.window_size:
            self.sample_list.append(data)
        else:
            # Add the new sample value to the beginning of the list
            self.sample_list.insert(0, data)
            # Remove last sample of the list
            self.sample_list.pop()
        
        # the FILTERING IS BELOW HERE
        cumulative_sum = sum(self.sample_list)
        return cumulative_sum / len(self.sample_list) / self.normalization_factor # return the average divided by the normalization factor
    
    
    def filtered_emg(self): #return the current filtered value and takes the absolute value 
        
        # TODO: revamp this code - we cannot use one filter instance for all three signals
        # for i, emg in enumerate(emgs):
        #     self.emg_sensor_value[i] = abs(BP_gain * BP_filter.filter(emgs[i].read())) # returns filtered signal by applying filter function from Biquad class and takes the absolute value
        #     pc.set(i, self.emg_sensor_value[i])

        # Mikelis's filter design - tested on the 0th EMG analog readout
        pc.set(0, emg0.read())              # Raw EMG signal at Ch0
        femg0 = self.prefilter(emg0.read())
        femg0 = abs(femg0)
        pc.set(1, femg0)                    # Filtered EMG signal at Ch1
        femg0_avg = self.moving_average(femg0)
        pc.set(2, femg0_avg)                # Filtered MA EMG signal at Ch2

        pc.send()
        return femg0_avg    # TODO: the ouput is set to 0 for now, this should be updated later on