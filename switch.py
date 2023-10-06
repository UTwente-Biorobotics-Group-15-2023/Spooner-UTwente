from pyb import Switch

class BlueSwitch(object):

    def __init__(self):
        
        self.switch_value = 0
        self.switch = Switch()
        self.switch.callback(self.callback)
        return
    
    def callback(self):
        self.switch_value = 1
        return
    
    def value(self):
        return_value = self.switch_value
        self.switch_value = 0
        return return_value
    

