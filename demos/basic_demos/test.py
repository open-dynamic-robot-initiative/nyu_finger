import numpy as np
from nyu_finger_sim.nyu_finger_simulator import  NYUFingerSimulator

robot = NYUFingerSimulator()


P = 1.
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