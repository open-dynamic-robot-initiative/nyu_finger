/**
 * @file dgm_nyu_finger_double.hpp
 * @author Julian Viereck
 * @author Huaijiang Zhu
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2021, New York University.
 */

#pragma once

#include "nyu_finger/nyu_finger.hpp"
#include "mim_msgs/srv/joint_calibration.hpp"
#include "dynamic_graph_manager/dynamic_graph_manager.hpp"
#include "yaml_utils/yaml_cpp_fwd.hpp"

namespace nyu_finger
{
class DGMNYUFinger : public dynamic_graph_manager::DynamicGraphManager
{
public:
    /**
     * @brief DGMNYUFingerDouble is the constructor.
     */
    DGMNYUFingerDouble();

    /**
     * @brief ~DemoSingleMotor is the destructor.
     */
    ~DGMNYUFingerDouble();

    /**
     * @brief This function make also sure that the joint velocity do not exceed
     * a certain value
     */
    bool is_in_safety_mode();

    /**
     * @brief initialize_hardware_communication_process is the function that
     * initialize the hardware.
     */
    void initialize_hardware_communication_process();

    /**
     * @brief get_sensors_to_map acquieres the sensors data and feed it to the
     * input/output map
     * @param[in][out] map is the sensors data filled by this function.
     */
    void get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map);

    /**
     * @brief set_motor_controls_from_map reads the input map that contains the
     * controls and send these controls to the hardware.
     * @param map
     */
    void set_motor_controls_from_map(
        const dynamic_graph_manager::VectorDGMap& map);

    /**
     * @brief Ros callback for the callibration procedure. Warning the robot
     * will move to the next the joint index and back to "0" upon this call.
     * Be sure that no controller are running in parallel.
     *
     * @param req nothing
     * @param res True if everything went well.
     * @return true if everything went well.
     * @return false if something went wrong.
     */
    void calibrate_joint_position_callback(
        mim_msgs::srv::JointCalibration::Request::SharedPtr req,
        mim_msgs::srv::JointCalibration::Response::SharedPtr res);

    /**
     * @brief compute_safety_controls computes safety controls very fast in case
     * the dynamic graph is taking to much computation time or has crashed.
     */
    void compute_safety_controls();

private:
    /**
     * @brief Calibrate the robot joint position
     *
     * @param zero_to_index_angle is the angle between the theoretical zero and
     * the next positive angle.
     */
    void calibrate_joint_position(
        const solo::Vector3d& zero_to_index_angle);

    /**
     * Entries for the real hardware.
     */

    /**
     * @brief First NYUFinger robot.
     */
    nyu_finger::NYUFinger nyu_finger_0_;

    /**
     * @brief Second NYUFinger robot.
     */
    nyu_finger::NYUFinger nyu_finger_1_;

    /**
     * @brief ctrl_joint_torques_ the joint torques to be sent. Used in this
     * class to perform a local copy of the control. This is need in order
     * to send this copy to the solo::Solo class
     */
    nyu_finger::Vector6d ctrl_joint_torques_;

    /**
     * @brief Check if we entered once in the safety mode and stay there if so
     */
    bool was_in_safety_mode_;

    /**
     * @brief These are the calibration value extracted from the paramters.
     * They represent the distance between the theorical zero joint angle and
     * the next jont index.
     */
    nyu_finger::Vector6d zero_to_index_angle_from_file_;
};

}  // namespace nyu_finger