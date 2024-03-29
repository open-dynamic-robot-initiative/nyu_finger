/**
 * @file dgm_nyu_finger.hpp
 * @author Julian Viereck
 * @author Huaijiang Zhu
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2021, New York University.
 */

#pragma once

#include "nyu_finger/nyu_finger.hpp"
#include "dynamic_graph_manager/hardware_process.hpp"
#include "yaml_utils/yaml_cpp_fwd.hpp"

#ifdef BUILD_WITH_ROS
#include "mim_msgs/srv/joint_calibration.hpp"
#endif

namespace nyu_finger
{
class DGMNYUFinger : public dynamic_graph_manager::HardwareProcess
{
public:
    /**
     * @brief DemoSingleMotor is the constructor.
     */
    DGMNYUFinger();

    /**
     * @brief ~DemoSingleMotor is the destructor.
     */
    ~DGMNYUFinger();

    /**
     * @brief This function make also sure that the joint velocity do not exceed
     * a certain value
     */
    bool is_in_safety_mode();

    /**
     * @brief initialize_drivers is the function that
     * initialize the hardware.
     */
    virtual void initialize_drivers();

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

#ifdef BUILD_WITH_ROS
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
#endif

    /**
     * @brief Calibrate the robot joint position
     *
     * @param zero_to_index_angle is the angle between the theoretical zero and
     * the next positive angle.
     */
    void calibrate_joint_position(
        Eigen::Ref<nyu_finger::Vector3d> zero_to_index_angle);

    /**
     * @brief Calibrates the joints using the calibration data from the yaml file.
     */
    void calibrate_joint_position_from_yaml();

    /**
     * @brief compute_safety_controls computes safety controls very fast in case
     * the dynamic graph is taking to much computation time or has crashed.
     */
    void compute_safety_controls();

private:
    /**
     * Entries for the real hardware.
     */

    /**
     * @brief solo_ is the hardware drivers.
     */
    nyu_finger::NYUFinger nyu_finger_;

    /**
     * @brief ctrl_joint_torques_ the joint torques to be sent. Used in this
     * class to perform a local copy of the control. This is need in order
     * to send this copy to the solo::Solo class
     */
    nyu_finger::Vector3d ctrl_joint_torques_;

    /**
     * @brief ctrl_joint_positions_ the desired joint position for the PD
     * controller running on the udriver board.
     */
    nyu_finger::Vector3d ctrl_joint_positions_;

    /**
     * @brief ctrl_joint_velocities_ the desired joint velocity for the PD
     * controller running on the udriver board.
     */
    nyu_finger::Vector3d ctrl_joint_velocities_;

    /**
     * @brief joint_position_gains_ the P gains for the PD controller running
     * on the udriver board.
     */
    nyu_finger::Vector3d joint_position_gains_;

    /**
     * @brief ctrl_joint_torques_ the D gains for the PD controller running
     * on the udriver board.
     */
    nyu_finger::Vector3d joint_velocity_gains_;

    /**
     * @brief Check if we entered once in the safety mode and stay there if so
     */
    bool was_in_safety_mode_;

    /**
     * @brief These are the calibration value extracted from the paramters.
     * They represent the distance between the theorical zero joint angle and
     * the next jont index.
     */
    nyu_finger::Vector3d zero_to_index_angle_from_file_;
};

}  // namespace nyu_finger
