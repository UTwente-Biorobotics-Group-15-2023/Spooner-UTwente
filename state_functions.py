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
        self.motor_1.write(-0.9)
        if self.sensor_state.ks_one_value == 0:
            self.motor_1.write(0)
        # Turn on M2 at low speed until ks is reached
        self.motor_2.write(-0.7)
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
        # Show the motor angles
        print("Motor 1 angle: ", self.sensor_state.angle_motor_1)
        print("Motor 2 angle: ", self.sensor_state.angle_motor_2)

        # gets the 0th emg signal and costrains it to the range 0 to 1
        emg0 = self.sensor_state.emg_value[0]                 # hopefully not much out of the range 0 to 1
        emg0 = 0 if emg0 < 0 else 1 if emg0 > 1 else emg0     # let's make sure it's really 0 to 1
        #print(emg0)

        # We get the desired joint velocities (setpoint) from the EMG signal, for now only used for controlling y-axis
        # qdot_sp = Kinematics.get_qdot(self, self.sensor_state.angle_motor_1, self.sensor_state.angle_motor_2, (0, 0.5))

        # Get m1dot_sp and m2dot_sp from the qdot_sp (go from joint to motor velocities)
        # q2 = 1/4*np.pi + ma2 - ma1
        # q1 = ma1 =>> ma2 = q2 + q1 - 1/4*np.pi
        # dma2 = dq2 + dq1
        # m1dot_sp = qdot_sp[0]
        # m2dot_sp = qdot_sp[0] + qdot_sp[1]
        # m1dot = self.m1_previous - self.sensor_state.angle_motor_1 * self.frequency
        # m2dot = self.m2_previous - self.sensor_state.angle_motor_2 * self.frequency

        
        # TODO: here the controller has to do its part
        # > measure the real m1dot, m2dot and compare to the m1dot_sp and m2dot_sp
        # > get the difference (error)
        # > use the error in PID controller to output pwm signal to the motors (0 to 1)
        # > cap the pwm signal to 0.8 (motor safety factor)

        # implementation of controller
        # reference1 = m1dot 
        # measured1 = self.sensor_state.angular_velocity #still needs to be made in the sensor.py file but not done bc of potential merge conflicts
        # reference1 = m2dot 
        # measured2 = self.sensor_state.angular_velocity #still needs to be made in the sensor.py file but not done bc of potential merge conflicts
        # self.pid_m1.step(reference1, measured1)
        # self.pid_m1.step(reference2, measured2)
        # self.pid_m1.step()

        # self.motor_1.write()
        # self.motor_2.write()
        

        # TODO: check plus and minus definitions of pwm and m - the below are educated guesses
        # for motor one - negative pwm makes it move in positive m direction
        # for motor two - negative pwm makes it move in negative m direction

        # older test code
        # emg0 *= -0.8             # motor safety factor - let's not use more than 0.8 of max power
        # TODO: map the emg_value to pwm signal and send to the motors!
        #emg0 = self.sensor_state.emg_value[0]                 # hopefully 0 to 1
        #emg0 = 0 if emg0 < 0 else 1 if emg0 > 1 else emg0     # let's make sure it's 0 to 1
        #emg0 *= -0.85                                         # motor safety factor - let's not use mre than 0.6 of max power
        #print(emg0)
        # self.motor_1.write(emg0)  # TODO: uncomment for the motor to move

        # Compensation controller (motor 1)
        #self.c1 = 0.05
        #self.c2 = 0
        #self.c3 = 6

        #self.compensation_controller.change_coefficient(self.sensor_state.potmeter_value)
        #print(self.compensation_controller.c2)
        #self.compensated_PWM_value = self.compensation_controller.calculate_u(self.sensor_state.angle_motor_1, self.sensor_state.angle_motor_1_previous)
        #self.motor_1.write(self.compensated_PWM_value)
        # self.angle_previous_1 = self.angle_current_1
        # self.angle_current_1 = self.sensor_state.angle_motor_1
        # self.compensated_PWM_value = self.compensation_controller.calculate_u(self.angle_current_1, self.angle_previous_1)
        # print(self.compensated_PWM_value)
        # self.motor_1.write(self.compensated_PWM_value)

        ## IMPORTANT: leave this code as last in the main action
        self.m1_previous = self.sensor_state.angle_motor_1 
        self.m2_previous = self.sensor_state.angle_motor_2
      
        #self.time = self.n * self.ticker_period
        #self.pwm_value = 0.6 * np.sin(self.sin_step*self.n)
        #self.motor_1.write(self.pwm_value)
        #print(self.pwm_value)
        #self.n += 1

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
