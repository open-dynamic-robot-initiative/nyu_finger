import os.path
import numpy as np

from robot_properties_nyu_finger.config import (
    NYUFingerDoubleConfig0, NYUFingerDoubleConfig1)
from nyu_finger.nyu_finger_hwp_cpp import NYUFingerHWP

if __name__ == "__main__":
    finger0 = NYUFingerHWP()
    finger1 = NYUFingerHWP()

    finger0.initialize(NYUFingerDoubleConfig0.dgm_yaml_path)
    finger1.initialize(NYUFingerDoubleConfig1.dgm_yaml_path)

    finger0.run()
    finger1.run()

    print()
    input("Press enter to start calibration.")

    finger0.calibrate(np.zeros(3))
    finger1.calibrate(np.zeros(3))
