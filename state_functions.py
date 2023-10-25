from states import State
from pyb import LED
from motor import Motor
from compensation_controller import CompensationController

class StateFunctions(object):

    def __init__(self, robot_state, sensor_state, ticker_frequency):
        self.led_yellow = LED(2) # Yellow LED on Nucleo
        self.led_red = LED(3)   # RED LED on Nucleo
        self.robot_state = robot_state
        self.sensor_state = sensor_state

        ## Compensation controller
        self.compensation_controller = CompensationController(ticker_frequency)
        self.angle_previous_1 = 1
        self.angle_current_1 = 1


        ## Motors
        self.motor_1 = Motor(ticker_frequency, 1)
        self.motor_2 = Motor(ticker_frequency, 2)
        
        ## Callback states
        self.callbacks = {
            State.CALLIBRATE: self.callibrate,
            State.HOME: self.home,
            State.HOLD: self.hold,
            State.MOVE: self.move,
            State.EMERGENCY_STOP: self.emergency_stop,
        }
        return

    def callibrate(self):
        # The below if statement makes sure we only execute entry action once
        # basically - checks if the states differ between this and previous update
        if self.robot_state.is_changed():
            ## Entry action
            print('Lets callibrate the EMG input!')
            print('Please remain still and relaxed for 10 seconds')
            print('When the LED turns off, you should contract your muscles as hard as you can')
            print('for now you can press BlueSwitch to proceed to HOME state')
            self.robot_state.set(State.CALLIBRATE)
        ## Main action
        
        ## Exit guards
        if self.sensor_state.switch_value == 1:
            # TODO: uncomment the below and delete the substitute
            #self.robot_state.set(State.EMERGENCY_STOP)
            self.robot_state.set(State.HOME)

        # TODO: if callibration done, move to HOME state

        return
    
    def home(self):
        if self.robot_state.is_changed():
            ## Entry action
            print('press BlueSwitch if EMERGENCY state is needed')
            self.led_red.off()
            self.led_yellow.off()
            self.robot_state.set(State.HOME)
        ## Main action

        #Compensation controller (motor 1)
        self.angle_previous_1 = self.angle_current_1
        self.angle_current_1 = self.sensor_state.angle_motor_1
        self.compensated_PWM_value = self.compensation_controller.calculate_u(self.angle_current_1,self.angle_previous_1)
        print(self.compensated_PWM_value)
        self.motor_1.write(self.compensated_PWM_value)

        # TODO: write code to control the motors such that the arm moves to the home position

        if self.sensor_state.ks_one_value == 1:
            self.led_yellow.off()
        else:
            self.led_yellow.on()
        if self.sensor_state.ks_two_value == 1:
            self.led_red.off()
        else:
            self.led_red.on()
        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.led_red.off()
            self.led_yellow.off()
            print("!! emergency !!")
            self.robot_state.set(State.EMERGENCY_STOP)
        
        # TODO: uncomment the below when ready to test
        # if self.sensor_state.ks_one_value == 0 and self.sensor_state.ks_two_value == 0:
        #     self.led_red.off()
        #     self.led_yellow.off()
        #     print("ROBOT REACHED HOME POSITION")
        #     self.robot_state.set(State.HOLD)

        return
    
    def hold(self):
        if self.robot_state.is_changed():
            ## Entry action
            self.robot_state.set(State.HOLD)
        ## Main action

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.EMERGENCY_STOP)

        # TODO: add a state trasition to MOVE if CONTINUE signal is detected

        return
    
    def move(self):
        if self.robot_state.is_changed():
            ## Entry action
            self.robot_state.set(State.MOVE)
        ## Main action

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.EMERGENCY_STOP)

        # TODO: add a state trasition to HOLD if the HOLD signal is detected

        return
    
    def emergency_stop(self):
        if self.robot_state.is_changed():
            ## Entry action
            print("entered EMERGENCY state")
            self.robot_state.set(State.EMERGENCY_STOP)
            self.motor_1.write(0)
        ## Main action

        ## Exit guards

        # NO EXIT GUARDS as this is a final state - reboot robot to restart

        return
