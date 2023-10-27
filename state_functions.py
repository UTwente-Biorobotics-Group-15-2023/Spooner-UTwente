from states import State
from motor import Motor
from compensation_controller import CompensationController

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
        self.calibration_sum = 0

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
            print('Lets CALIBRATE the EMG input!')
            print('Please CONTRACT YOUR MUSCLES as hard as you can')
            self.robot_state.set(State.CALLIBRATE)

        ## Main action
        # self.max_emg_pre_calibration = max([self.max_emg_pre_calibration, self.sensor_state.emg_value])
        # self.calibration_timer += 1

        self.calibration_timer += 1
        if self.calibration_timer > self.frequency * 2: # wait two seconds, then start collecting emg data
            self.calibration_sum += self.sensor_state.emg_value

        ## Exit guards
        if self.calibration_timer >= self.frequency * self.calibration_time:
            self.calibration_timer = 0
            print("CALIBRATION COMPLETE!")
            print(self.calibration_sum/(self.frequency * self.calibration_time))
            self.sensor_state.set_calibration_coefficient(self.calibration_sum/(self.frequency * (self.calibration_time-2)))
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

        # TODO: write code to control the motors such that the arm moves to the home position by itself

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            print("!! emergency !!")
            self.robot_state.set(State.EMERGENCY_STOP)

        if self.sensor_state.ks_one_value == 0 and self.sensor_state.ks_two_value == 0:
            print("ROBOT REACHED HOME POSITION")
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

        # TODO: map the emg_value to pwm signal and send to the motors!
        emg0 = self.sensor_state.emg_value[0]                 # hopefully 0 to 1
        emg0 = 0 if emg0 < 0 else 1 if emg0 > 1 else emg0     # let's make sure it's 0 to 1
        emg0 *= -0.85                                         # motor safety factor - let's not use mre than 0.6 of max power
        print(emg0)
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
