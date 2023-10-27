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
        self.motor_1 = Motor(18000, 1)
        self.motor_2 = Motor(18000, 2)

        ## Callback states
        self.callbacks = {
            State.CALLIBRATE: self.calibrate,
            State.HOME: self.home,
            State.HOLD: self.hold,
            State.MOVE: self.move,
            State.EMERGENCY_STOP: self.emergency_stop,
        }
        return

    ##State
    def calibrate(self):
        # The below if statement makes sure we only execute entry action once
        # basically - checks if the states differ between this and previous update
        if self.robot_state.is_changed():
            ## Entry action
            print('Lets calibrate the EMG input!')
            print('Please remain still and relaxed for 10 seconds')
            print('When the LED turns off, you should contract your muscles as hard as you can')
            print('for now you can press BlueSwitch to proceed to HOME state')
            self.robot_state.set(State.CALLIBRATE)

        ## Main action
        #TODO: Create calibration code

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.HOME)
        return

    def home(self):
        if self.robot_state.is_changed():
            ## Entry action
            print('press BlueSwitch if EMERGENCY state is needed')
            self.led_red.off()
            self.led_yellow.off()
            self.robot_state.set(State.HOME)
            print("HOME")
        ## Main action
        print(self.sensor_state.ks_one_value)
        print(self.sensor_state.ks_two_value)


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

        if self.sensor_state.ks_one_value == 1 and self.sensor_state.ks_two_value == 1:
            self.led_red.off()
            self.led_yellow.off()
            print("ROBOT REACHED HOME POSITION")
            self.robot_state.set(State.HOLD)
        return

    def hold(self):
        if self.robot_state.is_changed():
            ## Entry action
            self.robot_state.set(State.HOLD)
            print("HOLD")

        ## Main action
        # Kill the motors
        self.motor_1.write(0)

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.EMERGENCY_STOP)

        # TODO: Testing switching states with the kill switches, create a CONTINUE signal to transition to MOVE if CONTINUE signal is detected
        if self.sensor_state.ks_one_value == 1:# and self.sensor_state.ks_two_value == 1:
            self.robot_state.set(State.MOVE)
        return

    def move(self):
        if self.robot_state.is_changed():
            ## Entry action
            self.robot_state.set(State.MOVE)
            print("MOVE")

        ## Main action

        # Compensation controller (motor 1)
        #self.c1 = 0.05
        #self.c2 = 0
        #self.c3 = 6

        self.compensation_controller.change_coefficient(self.sensor_state.potmeter_value)
        self.angle_previous_1 = self.angle_current_1
        self.angle_current_1 = self.sensor_state.angle_motor_1
        self.compensated_PWM_value = self.compensation_controller.calculate_u(self.angle_current_1, self.angle_previous_1)
        print(self.compensated_PWM_value)
        self.motor_1.write(self.compensated_PWM_value)

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.EMERGENCY_STOP)
        #TODO: Testing switching states with the kill switches, create a HOLD signal
        elif self.sensor_state.ks_one_value == 0:# and self.sensor_state.ks_two_value == 0:
            self.robot_state.set(State.HOLD)
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
