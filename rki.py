from ulab import numpy as np
from diff_geom import tilde, Adjoint, inverseH, expT

## Robot variables
L1 = 0.3
L2 = 0.3

# Unit twist for reference position (brocket) used in get_H
T1 = np.array([1,0,0])
T2 = np.array([1,L1,0])

# He0 for q=0, used in get_H
He00 = np.eye(3)
He00[0,2] = 0       # origin x
He00[1,2] = L1+L2   # origin y

# control parameters
Kv = 7
vmax = 10

class Kinematics(object):
    
    def __init__(self):

        return
    
    def get_q(self, motor_angle_1, motor_angle_2):
        """
        =INPUT= motor_angle_1, motor_angle_2 input the angle obtained from the motor encoders (rad)

        =OUTPUT= get the angle for the virtual robot arm q1, q2 (rad)
        """
        q1 = motor_angle_1
        q2 = 1/4*np.pi + motor_angle_2 - motor_angle_1 # minus might change into plus depending on the reference configuration
        return q1, q2
    
    # Forward kinematics
    def get_H(self, q1, q2): # get the H matrix of EE to 0

        T1 = tilde(T1) #Turn twist into tilde/matrix form
        T2 = tilde(T2) #Turn twist into tilde/matrix form

        He0 = expT(T1)*q1 # Take the matrix exponential of T1
        He0 = He0.dot(expT(T2) * q2) # Take the matrix exponential of T2 and dot prod of e^T1 @ e^T2
        He0 = He0.dot(He00) # Take the dot product of e^T1 @ e^T2 @ H(0) completing the brocket equation

        return He0
    
    def get_J(self, q1): # Calculate the Jacobian using the modified jacobien method
        """
        =INPUT= q1 of "virtual" robot angle
        =OUTPUT= Modified / reduced Jacobian
        """
        J = np.zeros((3,2))
        J[:,0] = np.array([1,0,0], dtype = float)
        J[:,1] = np.array([1,L1*np.cos(q1),-(-L1*np.sin(q1))], dtype = float)        
        return J
    
    def get_qdot(self, motor_angle_1, motor_angle_2, v): 
        """
        =INPUT= motor_angle_1, motor_angle_2 are the motor angles measured, They are converted to the angles of the "virtual" joint
        =OUTPUT= gives the required angular velocity for the joints 
        """

        q1, q2 = self.get_q(motor_angle_1, motor_angle_2)


        He0 = self.get_H(q1, q2) # calculate the H EE to 0 matrix
        J = self.get_J(q1) # calculate the modified / reduced Jacobian

        pe0 = He0[:2,2] # grab the position of the EE from the H matrix

        ## Modified Jacobian method
        # v = Kv * (set_point - pe0)
        # v[v>vmax] = vmax
        # v[v<-vmax] = -vmax

        v = vmax * v

        H0f = np.eye(3)
        H0f[:2,2] = -pe0 # calculate H 0 to f (frame f is attached to EE frame, but differs in that it is non rotating, always level wrt ground )

        Jp = Adjoint(H0f) @ J
        Jpp = Jp[1:,:]  # taking out the row corresponding to the rotations

        try:
            Jppinv = np.linalg.inv(Jpp)
        except np.linalg.LinAlgError:
            print('matrix is singular')
            Jpp += 0.05 * np.eye(2)
            Jppinv = np.linalg.inv(Jpp)


        qdot = Jppinv @ v

        ## Calculate the angle using euler integration
        # if dt > 0.2:
        #     # limit to 200 ms: if more time passed, we would get massively unstable behaviour
        #     # (this sometimes happens for the first timestep)
        #     dt = 0.2
        # qs += dq * dt  
        return qdot
    
    def get_EE_position(self):

        return
    

    
