from switch import BlueSwitch
from switch import KillSwitchOne, KillSwitchTwo
from emg_sensor import EmgSensor
from potmeter import PotMeter

class SensorState(object): # this class keeps track of sensor data

    def __init__(self, serial_pc):

        ## Serial PC

        ## Blue switch
        self.switch_value = 0
        self.blue_switch = BlueSwitch()

        ## Kill switches
        self.ks_one_value = 0  # switch 1 for finding the arm's home state
        self.ks_two_value = 0  # switch 2 for finding the arm's home state
        self.kill_switch_one = KillSwitchOne()
        self.kill_switch_two = KillSwitchTwo()

        ## Emg sensor
        self.emg_sensor = EmgSensor(serial_pc)

        ## Potmeter
        self.potmeter = PotMeter() 
        self.potmeter_value = self.potmeter.value() #potmeter value with [pot1, pot2]

        return
    


    
    def update(self):

        ## Blue switch
        self.switch_value = self.blue_switch.value()

        ## Kill switch
        self.ks_one_value = self.kill_switch_one.value()
        self.ks_two_value = self.kill_switch_two.value()

        ## Emg sensor
        #self.emg_sensor = self.emg_sensor.moving_av() # gives the current value of the emg sensor (the moving average!) in vector form with [value emg1, value emg2, value emg3]
        
        ## Encoder
        # self.encoder_1 = Encoder(Pins.MOTOR_1_ENCODER_1, Pins.MOTOR_1_ENCODER_2)
        # self.encoder_2 = Encoder(Pins.MOTOR_2_ENCODER_1, Pins.MOTOR_2_ENCODER_2)
        # self.encoder_1_value = self.encoder_1.counter()
        # self.encoder_2_value = self.encoder_2.counter()
        return