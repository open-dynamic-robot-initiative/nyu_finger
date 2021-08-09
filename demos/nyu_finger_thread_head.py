# Author: Julian Viereck
# Date: July 21, 2021
#
# Basic example demonstrating the use of thread_head for a single finger robot.
#
# To run this script launch a ipython terminal and run the script
# 
# source setup.bash
# ipython
# $ run nyu_finger_thread_head.py.

import os.path
import numpy as np
import time
import dynamic_graph_manager_cpp_bindings
from dynamic_graph_head import ThreadHead, HoldPDController

from robot_properties_nyu_finger.config import NYUFingerConfig

# Specify the setup through a yaml file.
yaml_file = os.path.join(
    NYUFingerConfig.dgm_yaml_dir, 'dgm_parameters_nyu_finger.yaml')

# Create the dgm communication to the control process.
head = dynamic_graph_manager_cpp_bindings.DGMHead(yaml_file)

P = 3.
D = 0.05

# The hold_pd controller holds the current position when its activated.
# This controller is used as safety controller.
hold_pd_controller = HoldPDController(head, P, D)

# Example controller class.
class PDController:
    def __init__(self, head, Kp, Kd):
        self.head = head

        self.Kp = Kp
        self.Kd = Kd

        self.joint_positions = head.get_sensor('joint_positions')
        self.joint_velocities = head.get_sensor('joint_velocities')

    def warmup(self, thread_head):
        self.initial_position = self.joint_positions.copy()

    def run(self, thread_head):
        tau = (
            self.Kp * (self.initial_position - self.joint_positions) -
            self.Kd * self.joint_velocities
        )
        self.head.set_control('ctrl_joint_torques', tau)

# Instantiation of the PD controller:
pd_ctrl = PDController(head, P, D)


# The actual thread_head object which does the main control and safety handling.
thread_head = ThreadHead(
    0.001, # dt
    hold_pd_controller, # safety controller
    head, # thread heads
    []    # utils
)

# Start the parallel processing.
thread_head.start()

# From ipython terminal, you might want to run one of the following commands:
# 
# $ thread_head.switch_controllers(pd_ctrl) # activates the D controller
#
# $ thread_head.start_logging(5) # Logs the current controller for 5 seconds.
