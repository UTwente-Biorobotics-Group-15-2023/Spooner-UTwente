from biorobotics import SerialPC

#pc = SerialPC(4)

# PID controller class
class PID(object):

    def __init__(self, t_step, kp, ki, kd):
        """
        =INPUT=
            t_step - float
                controller time step
            kp, ki, kd
                proportional, integral, differential controller gain
        
        """
        self.t_step = t_step
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.error = 0
        self.integrated_error = 0
        return
    
    def step(self, reference, measured, filtfun=None): # calculates the output of the PID for 1 step
        
        '''
        =INPUT= input is the reference and the measured signal that needs to be controlled
        =OUTPUT= output is 1 step of the PID controller 
        '''
        
        current_error = reference - measured
        self.integrated_error += current_error * self.t_step #calculate the integrated error previous + current
        error_change = (current_error - self.error) / self.t_step #calculate the change in error 
        self.error = current_error # update the self.error error
        
        output_PID = self.kp * current_error + self.ki * self.integrated_error + self.kd * error_change

        # pc.set(0, measured)
        # pc.set(1, reference)
        # pc.set(2, current_error)
        # pc.set(3, output_PID)
        # pc.send()

        return output_PID