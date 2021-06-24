import os.path
import numpy as np
import time
import dynamic_graph_manager_cpp_bindings

from robot_properties_nyu_finger.config import NYUFingerConfig

# Specify the setup through a yaml file.
yaml_file = os.path.join(NYUFingerConfig.dgm_yaml_dir, 'dgm_parameters_nyu_finger.yaml')

# Create the dgm communication to the control process.
head = dynamic_graph_manager_cpp_bindings.DGMHead(yaml_file)

P = 3.
D = 0.05
dt = 0.001
next_time = time.time() + dt
do_control = True

# Get a reference to the joint positions and velocities.
joint_positions = head0.get_sensor('joint_positions')
joint_velocities = head0.get_sensor('joint_velocities')

# In your control loop:
while (do_control):
    if time.time() >= next_time:
        next_time += dt

        ###
        # Get the latest measurements from the shared memory.
        head.read()

        ###
        # A very simple example PD controller around the origin
        tau = -P * joint_positions + -D * joint_velocities
        head.set_control('ctrl_joint_torques', tau)

        ###
        # Write the results into shared memory again.
        head.write()

    time.sleep(0.0001)
