import os.path
import numpy as np

from robot_properties_nyu_finger.config import NYUFingerConfig
from nyu_finger.nyu_finger_hwp_cpp import NYUFingerHWP

if __name__ == "__main__":
    finger0 = NYUFingerHWP()
    finger1 = NYUFingerHWP()

    finger0.initialize(os.path.join(
        NYUFingerConfig.dgm_yaml_dir, 'dgm_parameters_nyu_finger_double_0.yaml'))
    finger1.initialize(os.path.join(
        NYUFingerConfig.dgm_yaml_dir, 'dgm_parameters_nyu_finger_double_1.yaml'))

    finger0.run()
    finger1.run()

    print()
    input("Press enter to start calibration.")

    finger0.calibrate(np.zeros(3))
    finger1.calibrate(np.zeros(3))
