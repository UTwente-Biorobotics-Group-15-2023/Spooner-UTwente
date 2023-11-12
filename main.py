from state_machine import StateMachine

frequency = 300  # Hz
robot = StateMachine(frequency)  # input ticker frequency at which robot will function
robot.start()
