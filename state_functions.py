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
        self.frequency = ticker_frequency

        # EMG calibration
        self.max_emg_pre_calibration = 0
        self.calibration_timer = 0
        self.calibration_time = 10 # in seconds

        ## Compensation controller
        self.compensation_controller = CompensationController(ticker_frequency)
        self.angle_previous_1 = 1
        self.angle_current_1 = 1

        ## Motors
        self.motor_1 = Motor(ticker_frequency, 1)
        self.motor_2 = Motor(ticker_frequency, 2)

        ## Callback states
        self.callbacks = {
            State.CALLIBRATE: self.calibrate,
            State.HOME: self.home,
            State.HOLD: self.hold,
            State.MOVE: self.move,
            State.EMERGENCY_STOP: self.emergency_stop,
        }
        return

    ## State CALIBRATE
    def calibrate(self):
        # The below if statement makes sure we only execute entry action once
        # basically - checks if the states differ between this and previous update
        if self.robot_state.is_changed():
            ## Entry action
            print('Lets calibrate the EMG input!')
            print('Please contract your muscles as hard as you can for a few seconds')
            print('When the robot begins to move, you can stop')
            self.robot_state.set(State.CALLIBRATE)

        ## Main action
        self.max_emg_pre_calibration = max([self.max_emg_pre_calibration, self.sensor_state.emg_value])
        self.calibration_timer += 1

        ## Exit guards
        if self.calibration_timer >= self.frequency * self.calibration_time:
            self.calibration_timer = 0
            self.sensor_state.set_calibration_coefficient(self.max_emg_pre_calibration)
            self.robot_state.set(State.HOME)
        return

    ## State HOME
    def home(self):
        if self.robot_state.is_changed():
            ## Entry action
            print('press BlueSwitch if EMERGENCY state is needed')
            self.led_red.off()
            self.led_yellow.off()
            self.robot_state.set(State.HOME)
            print("HOME")
            print("Move the robot to its home state to continue")
        ## Main action

        # TODO: write code to control the motors such that the arm moves to the home position by itself

        # The below code indicates the status of the kill-switches using on-Nucleo LED's
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

        if self.sensor_state.ks_one_value == 0 and self.sensor_state.ks_two_value == 0:
            self.led_red.off()
            self.led_yellow.off()
            print("ROBOT REACHED HOME POSITION")
            # TODO: switch this back after testing
            # self.robot_state.set(State.HOLD)
            self.robot_state.set(State.MOVE)
        return

    ## State HOLD
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

    ## State MOVE
    def move(self):
        if self.robot_state.is_changed():
            ## Entry action
            self.robot_state.set(State.MOVE)
            print("MOVE")

        ## Main action

        # TODO: map the emg_value to pwm signal and send to the motors!
        emg0 = self.sensor_state.emg_value # hopefully 0 to 1
        emg0 *= 0.3 # motor safety factor
        self.motor_1.write(emg0)

        # Compensation controller (motor 1)
        # self.angle_previous_1 = self.angle_current_1
        # self.angle_current_1 = self.sensor_state.angle_motor_1
        # self.compensated_PWM_value = self.compensation_controller.calculate_u(self.angle_current_1, self.angle_previous_1)
        # print(self.compensated_PWM_value)
        # self.motor_1.write(self.compensated_PWM_value)

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.EMERGENCY_STOP)
        #TODO: Testing switching states with the kill switches, create a HOLD signal
        elif self.sensor_state.ks_one_value == 0:# and self.sensor_state.ks_two_value == 0:
            self.robot_state.set(State.HOLD)
        return

    ## State EMERGENCY_STOP
    def emergency_stop(self):
        if self.robot_state.is_changed():
            ## Entry action
            print("entered EMERGENCY state")
            self.robot_state.set(State.EMERGENCY_STOP)
            self.motor_1.write(0)
            self.motor_2.write(0)
        ## Main action - NO MAIN ACTION I GUESS (?)
        ## Exit guards - NO EXIT GUARDS as this is a final state - reboot robot to restart
        return
