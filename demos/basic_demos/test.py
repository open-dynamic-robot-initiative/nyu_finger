import time
import numpy as np
import matplotlib.pylab as plt
from nyu_finger import NYUFingerReal

robot = NYUFingerReal()
robot.initialize('enp5s0f1')


P = 1.5
D = 0.05
des_pos = np.array([0, -0.6, 1.2])

while True:
    # Get the current robot state.
    q, v = robot.get_state()
    
    # PD Controller.
    tau = P * (des_pos - q) - D * v
    robot.send_joint_torque(tau)
    
    # This steps the simulator and waits for 1 ms.
    robot.step()      