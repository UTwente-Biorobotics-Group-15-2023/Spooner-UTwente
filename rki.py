import numpy as np
from diff_geom import tilde, Adjoint, inverseH, expT
## Robot variables

# y value of first joint (y1), second joint (y2), end effector (y3) in ref position expressed in frame 0 (meter)
y1 = 0.18564 
y2 = 0.485 
y3 = 0.55548 

# x value of joint 1 (x1), joint 2 (x2), end effector position (x3) in ref position expressed in frame 0 (meter)
x1 = -0.1176 
x2 = -0.138 
x3 = 0.1176 

# He0 (q = 0)
H_e_0_0 = np.array([[1, 0, x3],
                    [0, 1, y3],
                    [0, 0, 1]]) 

class Kinematics(object):
    
    def __init__(self):

        return
    
    def get_q(self, motor_angle_1, motor_angle_2):
        """
        =INPUT= motor_angle_1, motor_angle_2 input the angle obtained from the motor encoders

        =OUTPUT= get the angle for the virtual robot arm q1, q2
        """
        q1 = motor_angle_1
        q2 = motor_angle_1 - motor_angle_2 # minus might change into plus depending on the reference configuration
        return q1, q2
    
    def get_H(self, motor_angle_1, motor_angle_2): # get the H matrix of EE to 0

        q1 = self.get_q(motor_angle_1, motor_angle_2)[0]
        q2 = self.get_q(motor_angle_1, motor_angle_2)[1]
       
        T_1 =  np.array([1, y1, -x1]) #Twist of joint 1 in reference position
        T_2 =  np.array([1, y2, -x2]) #Twist of joint 2 in reference position

        T_1 = tilde(T_1) #Turn twist into tilde/matrix form
        T_2 = tilde(T_2) #Turn twist into tilde/matrix form

        H_e_0 = expT(T_1)*q1 # Take the matrix exponential of T1
        H_e_0 = H_e_0.dot(expT(T_2) * q2) # Take the matrix exponential of T2 and dot prod of e^T1 @ e^T2
        H_e_0 = H_e_0.dot(H_e_0_0) # Take the dot product of e^T1 @ e^T2 @ H(0) completing the brocket equation

        return H_e_0
    
    def get_J(self, q): # Calculate the Jacobian using the modified jacobien method


        return
    
    def get_q(self):

        return
    
    def get_EE_position(self):

        return
    

    
