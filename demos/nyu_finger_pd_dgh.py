import os.path
import numpy as np
import time
import dynamic_graph_manager_cpp_bindings

from robot_properties_nyu_finger.config import NYUFingerConfig

# Specify the setup through a yaml file.
yaml_file = os.path.join(NYUFingerConfig.dgm_yaml_dir, 'dgm_parameters_nyu_finger.yaml')

# Create the dgm communication to the control process.
head = dynamic_graph_manager_cpp_bindings.DGMHead(yaml_file)

P = 3 * np.ones(3)
D = 0.2 * np.ones(3)
target = np.zeros(3)
dt = 0.001
next_time = time.time() + dt
do_control = True

# Get a reference to the joint positions and velocities.
joint_positions = head.get_sensor('joint_positions')
joint_velocities = head.get_sensor('joint_velocities')

# In your control loop:
while (do_control):
    if time.time() >= next_time:
        next_time += dt

        ###
        # Get the latest measurements from the shared memory.
        head.read()

        ###
        # Set PD target position and velocity.
        head.set_control('ctrl_joint_positions', target)
        head.set_control('ctrl_joint_velocities', target)

        ###
        # Set the P and D gains.
        head.set_control('ctrl_joint_position_gains', P)
        head.set_control('ctrl_joint_velocity_gains', D)

        ###
        # Write the results into shared memory again.
        head.write()

    time.sleep(0.0001)
