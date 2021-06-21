/**
 * \file nyu_finger.hpp
 * \author Julian Viereck
 * \date 29 July 2020
 * \copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 */

#pragma once

#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/robot.hpp>
#include <odri_control_interface/common.hpp>

namespace nyu_finger
{
typedef Eigen::Matrix<double, 3, 1> Vector3d;

enum NYUFingerState
{
    initial,
    ready,
    calibrate
};

/**
 * @brief Definition and drivers for the NYUFinger robot.
 *
 */
class NYUFinger
{
public:
    /**
     * @brief Solo is the constructor of the class.
     */
    NYUFinger();

    /**
     * @brief Initialize the robot by setting aligning the motors and calibrate
     * the sensors to 0.
     * @param if_name Interface for connection to hardware.
     */
    void initialize(const std::string &network_id, const Vector3d& motor_numbers);

    /**
     * @brief Sets the maximum motor current.
     */
    void set_max_current(const double& max_current);

    /**
     * @brief send_target_torques sends the target currents to the motors.
     */
    void send_target_joint_torque(
        const Eigen::Ref<Vector3d> target_joint_torque);

    /**
     * @brief acquire_sensors acquire all available sensors, WARNING !!!!
     * this method has to be called prior to any getter to have up to date data.
     */
    void acquire_sensors();

    /**
     * @brief Calibrate the joints by moving to the next joint index position.
     *
     * @param home_offset_rad This is the angle between the index and the zero
     * pose.
     * @return true in case of success.
     * @return false in case of failure.
     */
    bool calibrate(const Vector3d& home_offset_rad);

    /** @brief State of the solo robot. */
    NYUFingerState state_;

    /** @brief Controller to run the calibration procedure */
    std::shared_ptr<odri_control_interface::JointCalibrator> calib_ctrl_;

    /** @brief Indicator if calibration should start. */
    bool calibrate_request_;

    /**
     * Joint properties
     */

    /**
     * @brief get_motor_inertias in [kg m^2]
     * @return Motor inertias.
     */
    const Eigen::Ref<Vector3d> get_motor_inertias()
    {
        return motor_inertias_;
    }

    /**
     * @brief get_motor_torque_constants in []
     * @return Torque constants of each motor.
     */
    const Eigen::Ref<Vector3d> get_motor_torque_constants()
    {
        return motor_torque_constants_;
    }

    /**
     * @brief get_joint_gear_ratios
     * @return Joint gear ratios
     */
    const Eigen::Ref<Vector3d> get_joint_gear_ratios()
    {
        return joint_gear_ratios_;
    }

    /**
     * @brief get_max_torque
     * @return the max torque that has been hardcoded in the constructor of this
     * class. @todo Parametrize the maximum current via yaml or something else.
     */
    const Eigen::Ref<Vector3d> get_motor_max_current()
    {
        return motor_max_current_;
    }

    /**
     * Sensor Data
     */

    /**
     * @brief get_joint_positions
     * @return  the joint angle of each module
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Vector3d> get_joint_positions()
    {
        return joint_positions_;
    }

    /**
     * @brief get_joint_velocities
     * @return the joint velocities
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Vector3d> get_joint_velocities()
    {
        return joint_velocities_;
    }

    /**
     * @brief get_joint_torques
     * @return the joint torques
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Vector3d> get_joint_torques()
    {
        return joint_torques_;
    }

    /**
     * @brief get_joint_torques
     * @return the target joint torques
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.

    */
    const Eigen::Ref<Vector3d> get_joint_target_torques()
    {
        return joint_target_torques_;
    }

    /**
     * @brief get_joint_encoder_index
     * @return the position of the index of the encoders a the motor level
     * WARNING !!!!
     * The method <acquire_sensors>"()" has to be called
     * prior to any getter to have up to date data.
     */
    const Eigen::Ref<Vector3d> get_joint_encoder_index()
    {
        return joint_encoder_index_;
    }

    /**
     * Hardware Status
     */

    /**
     * @brief get_motor_enabled
     * @return This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    const std::array<bool, 3>& get_motor_enabled()
    {
        return motor_enabled_;
    }

    /**
     * @brief get_motor_ready
     * @return This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    const std::array<bool, 3>& get_motor_ready()
    {
        return motor_ready_;
    }

    /**
     * @brief get_motor_board_enabled
     * @return This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    const std::array<bool, 2>& get_motor_board_enabled()
    {
        return motor_board_enabled_;
    }

    /**
     * @brief get_motor_board_errors
     * @return This gives the status (enabled/disabled of the onboard control
     * cards).
     */
    const std::array<int, 2>& get_motor_board_errors()
    {
        return motor_board_errors_;
    }

    /**
     * @brief has_error
     * @return Returns true if the robot hardware has an error, false otherwise.
     */
    bool has_error() const
    {
        for (const auto& error_code : motor_board_errors_)
        {
            if (error_code != 0)
            {
                return true;
            }
        }
        return false;
    }

private:
    /**
     * Joint properties
     */

    /** @brief Motors inertia. */
    Vector3d motor_inertias_;
    /** @brief DCM motor torque constants. */
    Vector3d motor_torque_constants_;
    /** @brief joint gear ratios (9). */
    Vector3d joint_gear_ratios_;
    /** @brief Max appliable current before the robot shutdown. */
    Vector3d motor_max_current_;
    /** @brief Offset to the theoretical "0" pose. */
    Vector3d joint_zero_positions_;
    /** @brief Max joint torques (Nm) */
    Eigen::Array<double, 3, 1> max_joint_torques_;
    /** @brief  Security margin on the saturation of the control. */
    static const double max_joint_torque_security_margin_;

    /**
     * -------------------------------------------------------------------------
     * Hardware status
     */

    /**
     * @brief This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    std::array<bool, 3> motor_enabled_;

    /**
     * @brief This gives the status (enabled/disabled) of each motors using the
     * joint ordering convention.
     */
    std::array<bool, 3> motor_ready_;

    /**
     * @brief This gives the status
     * (enabled/disabled of the onboard control cards).
     */
    std::array<bool, 2> motor_board_enabled_;

    /**
     * @brief This gives the status
     * (enabled/disabled of the onboard control cards).
     */
    std::array<int, 2> motor_board_errors_;

    /**
        * Joint data
        */

    /**
     * @brief joint_positions_
     */
    Vector3d joint_positions_;
    /**
     * @brief joint_velocities_
     */
    Vector3d joint_velocities_;
    /**
     * @brief joint_torques_
     */
    Vector3d joint_torques_;
    /**
     * @brief joint_target_torques_
     */
    Vector3d joint_target_torques_;
    /**
     * @brief joint_encoder_index_
     */
    Vector3d joint_encoder_index_;

    /**
     * -------------------------------------------------------------------------
     * Additional data
     */

    /** @brief This is the name of the network: Left column in ifconfig output
     */
    std::string network_id_;

    /** @brief Map the joint id to the motor board id, @see NYUFinger description.
     */
    std::array<int, 3> map_joint_id_to_motor_board_id_;
    /** @brief Map the joint id to the motor port id, @see NYUFinger description.
     */
    std::array<int, 3> map_joint_id_to_motor_port_id_;

    /**
     * Drivers communication objects
     */

    /**
     * @brief Main board drivers.
     *
     * PC <- Ethernet/Wifi -> main board <- SPI -> Motor Board
     */
    std::shared_ptr<MasterBoardInterface> main_board_ptr_;

    /**
     * @brief Collection of joints for nyu finger.
     */
    std::shared_ptr<odri_control_interface::JointModules> joints_;

    /**
     * @brief The odri robot abstraction.
     */
    std::shared_ptr<odri_control_interface::Robot> robot_;
};

}  // namespace blmc_robots
