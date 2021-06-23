import os.path
import numpy as np

from robot_properties_nyu_finger.config import NYUFingerConfig

from nyu_finger.nyu_finger_hwp_cpp import NYUFingerHWP

if __name__ == "__main__":
    finger0 = NYUFingerHWP()
    finger1 = NYUFingerHWP()

    yaml_path_dir =

    finger0.initialize(os.path.join(
        yaml_path_dir, 'dgm_parameters_nyu_finger_double_0.yaml'))
    finger1.initialize(os.path.join(
        yaml_path_dir, 'dgm_parameters_nyu_finger_double_1.yaml'))

    finger0.run()
    finger1.run()

    input("Press enter to start calibration.")

    # TODO: Read these from the yaml file.
    # calibration_offsets = np.zeros(3)
    # finger0.calibrate(calibration_offsets)
    # finger1.calibrate(calibration_offsets)
