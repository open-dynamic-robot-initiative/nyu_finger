import time

from nyu_finger import py_nyu_finger 

class NYUFingerReal:
    def __init__(self):
        self.robot = py_nyu_finger.NYUFinger()
        self.last_sleep = time.time()

    def initialize(self, device):
        self.robot.initialize(device)

    def get_state(self):
        self.robot.acquire_sensors()
        return self.robot.get_joint_positions(), self.robot.get_joint_velocities()

    def send_joint_torque(self, tau):
        self.robot.send_target_joint_torque(tau)

    def step(self):
        sleep_duration = 0.001 - (time.time() - self.last_sleep)

        if sleep_duration > 0.:
            time.sleep(sleep_duration)
            self.last_sleep += sleep_duration
        else:
            self.last_sleep = time.time()
