import os.path
import numpy as np
import time

from robot_properties_nyu_finger.config import NYUFingerConfig
import dynamic_graph_manager_cpp_bindings

# Specify the setup through a yaml file.
yaml_file = os.path.join(
    NYUFingerConfig.dgm_yaml_dir, 'dgm_parameters_nyu_finger.yaml')

# Create the dgm communication to the control process.
head = dynamic_graph_manager_cpp_bindings.DGMHead(yaml_file)

P = np.zeros(3)
D = 0.3 * np.ones(3)
dt = 0.001
next_time = time.time() + dt
do_control = True

ti = 0

# In your control loop:
while (do_control):
    if time.time() >= next_time:
        next_time += dt

        ###
        # Get the latest measurements from the shared memory.
        head.read()

        ###
        # Set the P and D gains.
        head.set_control('ctrl_joint_position_gains', P)
        head.set_control('ctrl_joint_velocity_gains', D)

        if ti % 1000 == 0:
            print('finger.calibration [Rad]:', -head.get_sensor('joint_positions'))
            print('')

        ###
        # Write the results into shared memory again.
        head.write()
        ti += 1

    time.sleep(0.0001)
