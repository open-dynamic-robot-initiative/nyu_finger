import os.path
import numpy as np

from robot_properties_nyu_finger.config import NYUFingerConfig

from nyu_finger.nyu_finger_hwp_cpp import NYUFingerHWP

if __name__ == "__main__":
    finger = NYUFingerHWP()

    finger.initialize(NYUFingerConfig.dgm_yaml_path)

    finger.run()

    input("Press enter to start calibration.")

    finger.calibrate_from_yaml()
