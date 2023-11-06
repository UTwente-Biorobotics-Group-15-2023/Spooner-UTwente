from states import State
from motor import Motor
from compensation_controller import CompensationController
import rki
from pid import PID
from ulab import numpy as np
from encoderstats import EncoderStats 
from biorobotics import SerialPC

# Home state motor angles. Global since always constant
m1 = (-6) * np.pi/180
m2 = (-13) * np.pi/180 # q2 = 1/2*np.pi + ma2 - ma1 =>> ma2 = q2 + q1 - 1/2*np.pi => will equal -13deg at home state

pc = SerialPC(3)
class StateFunctions(object):

    def __init__(self, robot_state, sensor_state, ticker_frequency):
        # Creating local copies of the upstream variables
        self.robot_state = robot_state
        self.sensor_state = sensor_state
        self.frequency = ticker_frequency

        # EMG calibration
        self.max_emg_pre_calibration = 0
        self.calibration_timer = 0
        self.calibration_time = 10 # in seconds, min is 3
        self.calibration_sum_0 = 0
        self.calibration_sum_1 = 0
        self.calibration_sum_2 = 0

        ## Compensation controller
        self.compensation_controller = CompensationController(ticker_frequency)

        ## PID Stuff
        self.pid_m1 = PID(1/ticker_frequency, 40, 1, 5) #kp = 20.6897, ki = 114.9425, kd = 0.9310
        self.pid_m2 = PID(1/ticker_frequency, 40, 1, 5)
        self.m1_previous = 0
        self.m2_previous = 0

        ## Motors
        self.motor_1 = Motor(18000, 1)
        self.motor_2 = Motor(18000, 2)
        self.q_sp = np.array([m1, 1/2*np.pi + m2 - m1])
        self.t = 0

        ## Callback states
        self.callbacks = {
            State.CALLIBRATE: self.calibrate,
            State.HOME: self.home,
            State.HOLD: self.hold,
            State.MOVE: self.move,
            State.EMERGENCY_STOP: self.emergency_stop,
        }
        return
    
    def get_v(self):
         # get the 0th emg signal and costrain it to range 0 to 1
        emg0 = self.sensor_state.emg_value[0]                 # hopefully not much out of range 0 to 1
        emg0 = 0 if emg0 < 0.2 else 1 if emg0 > 1 else emg0     # let's make sure it's really 0 to 1

        emg1 = self.sensor_state.emg_value[1]
        emg1 = 0 if emg1 < 0.2 else 1 if emg1 > 1 else emg1     # let's make sure it's really 0 to 1
        
        emg2 = self.sensor_state.emg_value[2]
        emg2 = -1 if emg2 < 0.6 else 1                        # let's make it 1 or -1 to invert axis velocities

        # Get the desired joint velocities (setpoint) from the EMG
        v = np.array([emg0*emg2, emg1*emg2]) # be default make the arm slowly move backwards, but when emg present - move forward

        pc.set(0, emg0)
        pc.set(1, self.sensor_state.emg_value[2])
        pc.set(2, emg2)
        # pc.set(3, v[0])
        # pc.set(4, v[1])
        pc.send()
        
        return v

    ## State CALIBRATE
    def calibrate(self):
        # The below if statement makes sure we only execute entry action once
        # basically - checks if the states differ between this and previous update
        if self.robot_state.is_changed():
            ## Entry action
            print('Lets CALIBRATE the EMG input!')
            print('CONTRACT YOUR MUSCLES as hard as you can')
            self.robot_state.set(State.CALLIBRATE)

        ## Main action
        self.get_v()
        self.calibration_timer += 1
        if self.calibration_timer > self.frequency * 2: # wait two seconds, then start collecting emg data
            self.calibration_sum_0 += self.sensor_state.emg_value[0]
            self.calibration_sum_1 += self.sensor_state.emg_value[1]
            self.calibration_sum_2 += self.sensor_state.emg_value[2]

        ## Exit guards
        if self.calibration_timer >= self.frequency * self.calibration_time:
            self.calibration_timer = 0
            coef0 = self.calibration_sum_0/(self.frequency * (self.calibration_time-2))
            coef1 = self.calibration_sum_1/(self.frequency * (self.calibration_time-2))
            coef2 = self.calibration_sum_2/(self.frequency * (self.calibration_time-2))
            self.sensor_state.set_calibration_coefficients(coef0, coef1, coef2)
            print("CALIBRATION COMPLETE!")
            self.robot_state.set(State.HOME)
        return

    ## State HOME
    def home(self):
        if self.robot_state.is_changed():
            ## Entry action
            self.robot_state.set(State.HOME)
            print('Robot moving to HOME STATE.\nYou can always press the BlueSwitch for EMERGENCY STOP')
        
        ## Main action
        self.get_v()
        if self.sensor_state.ks_one_value == 0:
            self.motor_1.write(0)
        else:
            self.motor_1.write(-0.80) # Turn on M1 at low speed until ks is reached
        
        if self.sensor_state.ks_two_value == 0:
            self.motor_2.write(0)
        else:
            self.motor_2.write(-0.60) # Turn on M2 at low speed until ks is reached

        ## Exit guards
        if self.sensor_state.switch_value == 1:
            self.robot_state.set(State.EMERGENCY_STOP)

        if self.sensor_state.ks_one_value == 0 and self.sensor_state.ks_two_value == 0:
            self.motor_1.write(0)
            self.motor_2.write(0)
            print("ROBOT REACHED HOME POSITION")
            self.sensor_state.encoder_motor_1.set_angle(m1)
            self.sensor_state.encoder_motor_2.set_angle(m2)
            self.robot_state.set(State.HOLD)
        return

    ## State HOLD
    def hold(self):
        if self.robot_state.is_changed():
            ## Entry action
            self.robot_state.set(State.HOLD)
            print("ENTERED HOLD STATE")
            # Show the motor angles
            print("Motor 1 angle: ", self.sensor_state.angle_motor_1)
            print("Motor 2 angle: ", self.sensor_state.angle_motor_2)

        ## Main action
        self.get_v()
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
            # RECALIBRATE THE q_sp TO THE CURRENT POSITION before movement
            q1, q2 = rki.get_joint_angle(self.sensor_state.angle_motor_1, self.sensor_state.angle_motor_2)
            self.q_sp = np.array([q1, q2])
            print("MOVE")

        ## Main action

        # HOW THE BELOW CODE WORKS:
        # • we have desired ee velocity that is mapped from from the EMG signal
        # • we do the kinematics to get the desired joint velocity
        # • we map the desired joint velocity to the motor pwm and input it

        # self.sin_signal_velocity = -1 * np.sin(1/6 * np.pi * self.t)
        # self.t += 1/self.frequency                  # euler integration
        # v = np.array([self.sin_signal_velocity, self.sin_signal_velocity]) # diagonal sin signal

        # TODO: uncomment the below to control the robot using EMG again
        v = self.get_v()
        # v = np.array([-0.8, 0])
        qdot_sp = rki.get_qdot(self.sensor_state.angle_motor_1, self.sensor_state.angle_motor_2, v, self.frequency)
        self.q_sp += qdot_sp * 1/self.frequency     # euler integration
        # print('joint 1 desired: ',self.q_sp[0])
        # print('joint 2 desired: ',self.q_sp[1])

        # Get m1_sp and m2_sp from the q_sp (go from joint to motor angles)
        m1_sp = self.q_sp[0]
        m2_sp = self.q_sp[1] + self.q_sp[0] - 1/2*np.pi

        pid_out_1 = self.pid_m1.step(m1_sp, self.sensor_state.angle_motor_1)
        pid_out_2 = self.pid_m2.step(m2_sp, self.sensor_state.angle_motor_2)
        self.motor_1.write(pid_out_1)
        self.motor_2.write(pid_out_2)

        # serial output the angle errors
        
        q1, q2 = rki.get_joint_angle(self.sensor_state.angle_motor_1, self.sensor_state.angle_motor_2)
        
        # self.sin_signal_m1 = 0.5 + 0.45 * np.sin(1/2 * np.pi * self.t)
        # self.sin_signal_m2 = 0.15 * np.sin(1/2 * np.pi * self.t)
        # pid_out_1 = self.pid_m1.step(self.sin_signal_m1, self.sensor_state.angle_motor_1) #0.05*np.sin(2*np.pi*0.1*self.t) - np.pi/9
        # pid_out_2 = self.pid_m2.step(self.sin_signal_m2, self.sensor_state.angle_motor_2)

        # OLD EMG-TO-MOTOR TEST CODE
        # emg0 *= -0.8             # motor safety factor - let's not use more than 0.8 of max power
        # TODO: map the emg_value to pwm signal and send to the motors!
        #emg0 = self.sensor_state.emg_value[0]                 # hopefully 0 to 1
        #emg0 = 0 if emg0 < 0 else 1 if emg0 > 1 else emg0     # let's make sure it's 0 to 1
        #emg0 *= -0.85                                         # motor safety factor - let's not use mre than 0.6 of max power
        #print(emg0)
        # self.motor_1.write(emg0)  # TODO: uncomment for the motor to move

        # COMPENSATION CONTROLLER (motor 1)
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
