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
LP_10_low_0 =         Biquad(-1.4182,    0.5528,   0.7427,  -1.4855,    0.7427)   # 300Hz fs; 10Hz cut off Butterworth high pass
LP_48_52_stop_0 =     Biquad(-0.9556,    0.9253,   0.9626,  -0.9556,    0.9626)   # 300Hz fs; 48-52 band stop Butterworth
LP_98_102_stop_0 =    Biquad( 0.9855,    0.9439,   0.9855,   0.9439,    0.9711)   # 300Hz fs; 98-102 band stop Butterworth

LP_10_low_1 =         Biquad(-1.4182,    0.5528,   0.7427,  -1.4855,    0.7427)   # 300Hz fs; 10Hz cut off Butterworth high pass
LP_48_52_stop_1 =     Biquad(-0.9556,    0.9253,   0.9626,  -0.9556,    0.9626)   # 300Hz fs; 48-52 band stop Butterworth
LP_98_102_stop_1=    Biquad( 0.9855,    0.9439,   0.9855,   0.9439,    0.9711)   # 300Hz fs; 98-102 band stop Butterworth

LP_10_low_2 =         Biquad(-1.4182,    0.5528,   0.7427,  -1.4855,    0.7427)   # 300Hz fs; 10Hz cut off Butterworth high pass
LP_48_52_stop_2 =     Biquad(-0.9556,    0.9253,   0.9626,  -0.9556,    0.9626)   # 300Hz fs; 48-52 band stop Butterworth
LP_98_102_stop_2 =    Biquad( 0.9855,    0.9439,   0.9855,   0.9439,    0.9711)   # 300Hz fs; 98-102 band stop Butterworth

# putting the filters in one array
filters_0 = [LP_10_low_0, LP_48_52_stop_0, LP_98_102_stop_0]
filters_1 = [LP_10_low_1, LP_48_52_stop_1, LP_98_102_stop_1]
filters_2 = [LP_10_low_2, LP_48_52_stop_2, LP_98_102_stop_2]
filters = [filters_0, filters_1, filters_2]

emgs = [AnalogIn('A0'), AnalogIn('A1'), AnalogIn('A2')]
pc = SerialPC(3)

class EmgSensor(object): 


    def __init__(self):
        self.sample_lists = [[], [], []]
        self.normalization_factors = [1, 1, 1] # we determine this during calibration step, stronger person -> larger factor, begins at 1, then set to max emg value recorded during calibration step
        self.window_size = 80
        self.emg_sensor_value = [0, 0, 0] # initial value of EMG [sensor 1, sensor 2, sensor 3]
        return
    

    # Combination of multiple filters, Filtering 10Hz high pass to get rid of DC components, then filtering out 50Hz & 100Hz frequency peaks
    def prefilter(self, data, n=0):
        return filters[n][2].filter(filters[n][1].filter(filters[n][0].filter(data)))
    

    # Moving average filter - averages over the window size, returns the average
    def moving_average(self, data, n=0):
        # Get the correct sample list and normalization factor
        sample_list = self.sample_lists[n]
        normalization_factor = self.normalization_factors[n]

        # take care of the sample list
        if len(sample_list) < self.window_size:
            sample_list.append(data)
        else:
            sample_list.insert(0, data) # Add the new sample value to the beginning of the list
            sample_list.pop()           # Remove last sample of the list
        
        # the FILTERING IS BELOW HERE
        cumulative_sum = sum(sample_list)
        return cumulative_sum / len(sample_list) / normalization_factor # return the average divided by the normalization factor (max emg signal during calibration)
    
    
    # Function called within the SENSOR STATE UPDATE()
    def filtered_emg(self): #return the current filtered value and takes the absolute value 

        for i, emg in enumerate(emgs):
            femg = abs(self.prefilter(emg.read(), i))
            femg_avg = self.moving_average(femg, i)
            pc.set(i, femg_avg)
            self.emg_sensor_value[i] = femg_avg

        # Mikelis's filter design - tested on the 0th EMG analog readout
        # emg0 = AnalogIn('A0')
        # pc.set(0, emg0.read())              # Raw EMG signal at Ch0
        # femg0 = self.prefilter(emg0.read(), 0)
        # femg0 = abs(femg0)
        # pc.set(1, femg0)                    # Filtered EMG signal at Ch1
        # femg0_avg = self.moving_average(femg0, 0)
        # pc.set(2, femg0_avg)                # Filtered MA EMG signal at Ch2

        pc.send()
        return self.emg_sensor_value
    

    # Function called within the exit action of CALIBRATE state function
    def set_calibration_coefficients(self, coef0, coef1, coef2):
        self.normalization_factors = [coef0, coef1, coef2]
        return