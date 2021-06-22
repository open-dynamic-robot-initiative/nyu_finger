import numpy as np

from nyu_finger.nyu_finger_hwp_cpp import NYUFingerHWP

if __name__ == "__main__":
    finger0 = NYUFingerHWP()
    finger1 = NYUFingerHWP()

    yaml_path = '/home/jviereck/dev/dgm_refactoring/workspace/install/robot_properties_nyu_finger/lib/python3.6/site-packages/robot_properties_nyu_finger/robot_properties_nyu_finger/dynamic_graph_manager/'

    finger0.initialize(yaml_path + 'dgm_parameters_nyu_finger_double_0.yaml')
    finger1.initialize(yaml_path + 'dgm_parameters_nyu_finger_double_1.yaml')

    finger0.run()
    finger1.run()

    input("Press enter to start calibration.")

    # TODO: Read these from the yaml file.
    calibration_offsets = np.zeros(3)
    finger0.calibration(calibration_offsets)
    finger1.calibration(calibration_offsets)
