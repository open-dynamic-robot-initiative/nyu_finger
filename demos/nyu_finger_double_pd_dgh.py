import os.path
import numpy as np
import time

from robot_properties_nyu_finger.config import NYUFingerConfig
import dynamic_graph_manager_cpp_bindings

# Specify the setup through a yaml file.
yaml_file_0 = os.path.join(
    NYUFingerConfig.dgm_yaml_dir, 'dgm_parameters_nyu_finger_double_0.yaml')
yaml_file_1 = os.path.join(
    NYUFingerConfig.dgm_yaml_dir, 'dgm_parameters_nyu_finger_double_1.yaml')

# Create the dgm communication to the control process.
head0 = dynamic_graph_manager_cpp_bindings.DGMHead(yaml_file_0)
head1 = dynamic_graph_manager_cpp_bindings.DGMHead(yaml_file_1)

P = 3.
D = 0.05
dt = 0.001
next_time = time.time() + dt
do_control = True

# In your control loop:
while (do_control):
    if time.time() >= next_time:
        next_time += dt

        ###
        # Get the latest measurements from the shared memory.
        head0.read()
        head1.read()

        ###
        # Set the P and D gains.
        P = np.zeros(3)
        D = 0.2 * np.ones(3)

        head0.set_control('ctrl_joint_position_gains', P)
        head1.set_control('ctrl_joint_position_gains', P)

        head0.set_control('ctrl_joint_velocity_gains', D)
        head1.set_control('ctrl_joint_velocity_gains', D)

        ###
        # Write the results into shared memory again.
        head0.write()
        head1.write()

    time.sleep(0.0001)
