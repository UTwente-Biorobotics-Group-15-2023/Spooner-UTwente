from states import State
from motor import Motor
from compensation_controller import CompensationController
from rki import Kinematics
from pid import PID
from ulab import numpy as np
from encoderstats import EncoderStats 
# we have desired ee velocity that is mapped from from the EMG signal
# we do the kinematics to get the desired joint velocity
# we map the desired joint velocity to the motor pwm and input it

class StateFunctions(object):

    def __init__(self, robot_state, sensor_state, ticker_frequency):
        # Creating local copies of the upstream variables
        self.robot_state = robot_state
        self.sensor_state = sensor_state
        self.frequency = ticker_frequency

        # EMG calibration
        self.max_emg_pre_calibration = 0
        self.calibration_timer = 0
        self.calibration_time = 10 # in seconds
        self.calibration_sum_0 = 0
        self.calibration_sum_1 = 0
        self.calibration_sum_2 = 0

        self.T = 4

        self.ticker_period = 1/ticker_frequency
        self.sin_total_steps = self.T/self.ticker_period
        self.sin_step = 2*np.pi / self.sin_total_steps
        self.time = 0
        self.n = 0

        #self.sensor_state.

        ## Compensation controller
        self.compensation_controller = CompensationController(ticker_frequency)

        ## PID Stuff
        self.pid_m1 = PID(1/ticker_frequency, 1, 1, 1)
        self.pid_m2 = PID(1/ticker_frequency, 1, 1, 1)
        self.m1_previous = 0
        self.m2_previous = 0

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

    ## State CALIBRATE
    def calibrate(self):
        # The below if statement makes sure we only execute entry action once
        # basically - checks if the states differ between this and previous update
        if self.robot_state.is_changed():
            ## Entry action
            print('Lets CALIBRATE the EMG input!')
            print('Please CONTRACT YOUR MUSCLES as hard as you can')
            self.robot_state.set(State.CALLIBRATE)


        #TODO: fix the calibration, throws errors
        ## Main action
        # self.max_emg_pre_calibration = max([self.max_emg_pre_calibration, self.sensor_state.emg_value])
        # self.calibration_timer += 1

        self.calibration_timer += 1
        if self.calibration_timer > self.frequency * 2: # wait two seconds, then start collecting emg data
            self.calibration_sum_0 += self.sensor_state.emg_value[0]
            self.calibration_sum_1 += self.sensor_state.emg_value[1]
            self.calibration_sum_2 += self.sensor_state.emg_value[2]

        ## Exit guards
        if self.calibration_timer >= self.frequency * self.calibration_time:
            self.calibration_timer = 0
            print("CALIBRATION COMPLETE!")
            coef0 = self.calibration_sum_0/(self.frequency * (self.calibration_time-2))
            coef1 = self.calibration_sum_1/(self.frequency * (self.calibration_time-2))
            coef2 = self.calibration_sum_2/(self.frequency * (self.calibration_time-2))
            self.sensor_state.set_calibration_coefficients(coef0, coef1, coef2)
            self.robot_state.set(State.HOME)
        return

    ## State HOME
    def home(self):
        if self.robot_state.is_changed():
            ## Entry action
            self.robot_state.set(State.HOME)
            print("Move the robot to its home state to continue")
            print('press BlueSwitch for EMERGENCY state')
        ## Main action

        # Turn on M1 at low speed until ks is reached
        self.motor_1.write(0)
        if self.sensor_state.ks_one_value == 0:
            self.motor_1.write(0)
        # Turn on M2 at low speed until ks is reached
        self.motor_2.write(0)
        if self.sensor_state.ks_two_value == 0:
            self.motor_2.write(0)

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            print("!! emergency !!")
            self.motor_1.write(0)
            self.motor_2.write(0)
            self.robot_state.set(State.EMERGENCY_STOP)

        if self.sensor_state.ks_one_value == 0 and self.sensor_state.ks_two_value == 0:
            self.motor_1.write(0)
            self.motor_2.write(0)
            print("ROBOT REACHED HOME POSITION")
            
            #from feature/linear-movement-test
            # the home state angles might be somewhat off, needs to be checked
            # m1 = -6 * np.pi/180
            # q2 = 1/4*np.pi + ma2 - ma1 =>> ma2 = q2 + q1 - 1/4*np.pi
            # m2 = 83 * np.pi/180 + m1 - 1/4*np.pi # will equal -13 at home state
            # self.sensor_state.encoder_motor_1.set_home_angle(m1)
            # self.sensor_state.encoder_motor_2.set_home_angle(m2)
            
            self.sensor_state.encoder_motor_1.set_angle(356)
            self.sensor_state.encoder_motor_2.set_angle(81)
            self.robot_state.set(State.HOLD)
        return

    ## State HOLD
    def hold(self):
        if self.robot_state.is_changed():
            ## Entry action
            self.robot_state.set(State.HOLD)
            print("ENTERED HOLD STATE")

        ## Main action
        # Kill the motors
        self.motor_1.write(0)
        self.motor_2.write(0)
        # Show the motor angles
        print("Motor 1 angle: ", self.sensor_state.angle_motor_1)
        print("Motor 2 angle: ", self.sensor_state.angle_motor_2)

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.EMERGENCY_STOP)

        # TODO: Create a CONTINUE signal to transition to MOVE if CONTINUE signal is detected
        if self.sensor_state.ks_one_value == 1 and self.sensor_state.ks_two_value == 1:
            self.robot_state.set(State.MOVE)
        return

    ## State MOVE
    def move(self):
        if self.robot_state.is_changed():
            ## Entry action
            self.robot_state.set(State.MOVE)
            print("MOVE")

        ## Main action
        print("IN MOVE STATE")
        
        ## Make moter turn
        self.motor_2.write(1)
        print(self.sensor_state.angular_velocity_2)
        print(self.sensor_state.angle_motor_2_previous * 180 / np.pi)
        print(self.sensor_state.angle_motor_2 * 180 / np.pi) 
        
     
        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.EMERGENCY_STOP)
        #TODO: For now switching states only if any of the kill switches pressed, create an additional HOLD signal for future use
        elif self.sensor_state.ks_one_value == 0 or self.sensor_state.ks_two_value == 0:
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
        ## Main action - NO MAIN ACTION I GUESS
        ## Exit guards - NO EXIT GUARDS as this is a final state - reboot robot to restart
        return
