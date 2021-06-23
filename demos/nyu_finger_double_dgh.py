import numpy as np

import time

from robot_properties_nyu_finger.config import NYUFingerConfig

import dynamic_graph_manager_cpp_bindings

# Specify the setup through a yaml file.
yaml_file_0 = os.path.join(
    NYUFingerConfig.dgm_yaml_dir, 'dgm_parameters_nyu_finger_double_0.yaml')
yaml_file_0 = os.path.join(
    NYUFingerConfig.dgm_yaml_dir, 'dgm_parameters_nyu_finger_double_1.yaml')

# Create the dgm communication to the control process.
head0 = dynamic_graph_manager_cpp_bindings.DGMHead(yaml_file)
head1 = dynamic_graph_manager_cpp_bindings.DGMHead(yaml_file)

P = 3.
D = 0.05
dt = 0.001
next_time = time.time() + dt
do_control = True

target_pos = np.zeros(3)
offset = 0

c = 0

# In your control loop:
while (do_control):
    if time.time() >= next_time:
        c += 1
        next_time += dt

        ###
        # Get the latest measurements from the shared memory.
        head0.read()
        head1.read()

        # Get a reference to the joint velocities.
        joint_positions = head0.get_sensor('joint_positions')
        joint_velocities = head0.get_sensor('joint_velocities')

        # Run a simple PD controller.
        tau = P * (target_pos - joint_positions) + -D * joint_velocities
        head0.set_control('ctrl_joint_torques', tau)

        # Get a reference to the joint velocities.
        joint_positions = head1.get_sensor('joint_positions')
        joint_velocities = head1.get_sensor('joint_velocities')

        # Run a simple PD controller.
        tau = P * (target_pos - joint_positions) + -D * joint_velocities
        head1.set_control('ctrl_joint_torques', tau)

        ###
        # Write the results into shared memory again.
        head0.write()
        head1.write()

    time.sleep(0.0001)
