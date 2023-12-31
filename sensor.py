from switch import BlueSwitch, KillSwitchOne, KillSwitchTwo
from emg_sensor import EmgSensor
from potmeter import PotMeter
from encoderstats import EncoderStats

class SensorState(object): # this class keeps track of sensor data
  
    def __init__(self, ticker_frequency):
        ## Serial PC

        ## Blue switch
        self.switch_value = 0
        self.blue_switch = BlueSwitch()

        ## Kill switches
        self.ks_one_value = 1  # switch 1 for finding the arm's home state #TODO: change not pressed being 0
        self.ks_two_value = 1  # switch 2 for finding the arm's home state #TODO: change not pressed being 0
        self.kill_switch_one = KillSwitchOne() # default pin is "D70"
        self.kill_switch_two = KillSwitchTwo() # default pin is "D69" *nice*

        ## Emg sensor
        self.emg_sensor = EmgSensor()
        self.emg_value = 0

        ## Potmeter
        self.potmeter = PotMeter()
        self.potmeter_value = [0, 0] #potmeter value with [pot1, pot2]

        ## Encoder
        self.encoder_motor_1 = EncoderStats(1, ticker_frequency)
        self.encoder_motor_2 = EncoderStats(2, ticker_frequency)
        self.angle_motor_1 = 0
        self.angle_motor_2 = 0
        self.angle_motor_1_previous = 0
        self.angle_motor_2_previous = 0
        return
    
    def update(self):
        ## Blue switch
        self.switch_value = self.blue_switch.value()

        ## Potmeter update
        self.potmeter_value = self.potmeter.value()

        ## Kill switch
        self.ks_one_value = self.kill_switch_one.value()
        self.ks_two_value = self.kill_switch_two.value()

        ## Emg sensor
        self.emg_value = self.emg_sensor.filtered_emg()
        # self.emg_sensor = self.emg_sensor.moving_av() # gives the current value of the emg sensor (the moving
        # average!) in vector form with [value emg1, value emg2, value emg3]
        
        ## Encoder
        self.angle_motor_1_previous = self.angle_motor_1
        self.angle_motor_1 = self.encoder_motor_1.get_angle()
        self.angle_motor_2_previous = self.angle_motor_2
        self.angle_motor_2 = self.encoder_motor_2.get_angle()
        
        self.angular_velocity_1 = self.encoder_motor_1.get_angular_velocity(self.angle_motor_1, self.angle_motor_1_previous)
        self.angular_velocity_2 = self.encoder_motor_2.get_angular_velocity(self.angle_motor_1, self.angle_motor_1_previous)
        return
    
    def set_calibration_coefficients(self, coef0, coef1, coef2):
        self.emg_sensor.set_calibration_coefficients(coef0, coef1, coef2)
        return