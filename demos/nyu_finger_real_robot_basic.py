import time
import numpy as np
import matplotlib.pylab as plt
from nyu_finger import NYUFingerReal
from IPython.display import display, clear_output

np.set_printoptions(suppress=True, precision=3)

robot = NYUFingerReal()
robot.initialize('enp5s0f1')

P = 3.0
D = 0.05

des_pos = np.array([0.0,0.0, 0.0])
des_vel = np.array([0., 0., 0.])

traj_time = 2*10000

traj_des_pos = np.zeros((traj_time, 3))
traj_pos = np.zeros((traj_time, 3))

i = 0

start = time.time()
try:
    while True:
        t = time.time()
        des_pos[:] = 0.
        des_vel[:] = 0.
        
        q, v = robot.get_state()
        
        # Recording
        traj_des_pos[i] = des_pos
        traj_pos[i] = q

        tau = P * (des_pos - q) + D * (des_vel - v)

        robot.send_joint_torque(tau)
        
        robot.step()
        i += 1
except KeyboardInterrupt:
    print('interrupted!')

end = time.time()
    
robot.send_target_joint_torque(np.array([0., 0., 0.]))
