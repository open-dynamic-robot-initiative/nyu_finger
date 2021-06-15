#include "nyu_finger/nyu_finger.hpp"
#include <cmath>

#include "real_time_tools/spinner.hpp"

namespace nyu_finger
{

using namespace odri_control_interface;

NYUFinger::NYUFinger()
{
    /**
     * Hardware properties
     */
    motor_inertias_.setZero();
    motor_torque_constants_.setZero();
    joint_gear_ratios_.setZero();
    motor_max_current_.setZero();
    max_joint_torques_.setZero();
    joint_zero_positions_.setZero();
    reverse_polarities_.fill(false);
    slider_positions_vector_.resize(5);

    /**
     * Hardware status
     */
    for(unsigned i = 0 ; i < motor_enabled_.size(); ++i)
    {
        motor_enabled_[i] = false;
        motor_ready_[i] = false;
    }
    for(unsigned i = 0; i < motor_board_enabled_.size(); ++i)
    {
	    motor_board_enabled_[i] = false;
        motor_board_errors_[i] = 0;
    }

    /**
     * Joint data
     */
    joint_positions_.setZero();
    joint_velocities_.setZero();
    joint_torques_.setZero();
    joint_target_torques_.setZero();
    joint_encoder_index_.setZero();

    /**
     * Setup some known data
     */

    // for now this value is very small but it is currently for debug mode
    motor_max_current_.fill(2.); // TODO: set as paramters?
    motor_torque_constants_.fill(0.025);
    motor_inertias_.fill(0.045);
    joint_gear_ratios_.fill(9.0);

    state_ = Solo12State::initial;
}

void NYUFinger::initialize(const std::string &network_id)
{
    network_id_ = network_id;

    main_board_ptr_ = std::make_shared<MasterBoardInterface>(network_id_);

    VectorXi motor_numbers(3);
    motor_numbers << 0, 3, 2;
    VectorXb motor_reversed(3);
    motor_reversed << false, false, false;


    double HAA = 1.0;
    double HFE = 1.0;
    double KFE = 1.0;
    Eigen::VectorXd joint_lower_limits(3);
    Eigen::VectorXd joint_upper_limits(3);
    joint_lower_limits << -HAA, -HFE, -KFE;
    joint_upper_limits << HAA, HFE, KFE;

    // Define the joint module.
    joints_ = std::make_shared<odri_control_interface::JointModules>(
        main_board_ptr_,
        motor_numbers,
        motor_torque_constants_(0),
        joint_gear_ratios_(0),
        motor_max_current_(0),
        motor_reversed,
        joint_lower_limits,
        joint_upper_limits,
        20.,
        0.2);

    // Define the robot.
    robot_ = std::make_shared<odri_control_interface::Robot>(
        main_board_ptr_, joints_, nullptr /* imu */);

    // Define the calibration.
    std::vector<odri_control_interface::CalibrationMethod> directions{
        odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE,
        odri_control_interface::POSITIVE,
    };

    // Use zero position offsets for now. Gets updated in the calibration
    // method.
    Eigen::VectorXd position_offsets(3);
    position_offsets.fill(0.);
    calib_ctrl_ = std::make_shared<odri_control_interface::JointCalibrator>(
        robot_->joints, directions, position_offsets, 5., 0.05, 1.0, 0.001);

    // Initialize the robot.
    robot_->Init();
}

void NYUFinger::acquire_sensors()
{
    robot_->ParseSensorData();

    auto joints = robot_->joints;
    auto imu = robot_->imu;

    /**
     * Joint data
     */
    // acquire the joint position
    joint_positions_ = joints->GetPositions();
    // acquire the joint velocities
    joint_velocities_ = joints->GetVelocities();
    // acquire the joint torques
    joint_torques_ = joints->GetMeasuredTorques();
    // acquire the target joint torques
    joint_target_torques_ = joints->GetSentTorques();

    /**
     * The different status.
     */

    // motor board status
    ConstRefVectorXi motor_board_errors = joints->GetMotorDriverErrors();
    ConstRefVectorXb motor_driver_enabled = joints->GetMotorDriverEnabled();
    for (int i = 0; i < 2; i++)
    {
        motor_board_errors_[i] = motor_board_errors[i];
        motor_board_enabled_[i] = motor_driver_enabled[i];
    }

    // motors status
    ConstRefVectorXb motor_enabled = joints->GetEnabled();
    ConstRefVectorXb motor_ready = joints->GetReady();
    for (int i = 0; i < 3; i++)
    {
        motor_enabled_[i] = motor_enabled[i];
        motor_ready_[i] = motor_ready[i];
    }
}

void NYUFinger::set_max_joint_torques(const double& max_joint_torques)
{
    robot_->joints->SetMaximumCurrents(max_current);
}

void NYUFinger::send_target_joint_torque(
    const Eigen::Ref<Vector3d> target_joint_torque)
{
robot_->joints->SetTorques(target_joint_torque);

    switch (state_)
    {
        case NYUFingerState::initial:
            robot_->joints->SetZeroCommands();
            if (!robot_->IsTimeout() && !robot_->IsAckMsgReceived())
            {
                robot_->SendInit();
            }
            else if (!robot_->IsReady())
            {
                robot_->SendCommand();
            }
            else
            {
                state_ = NYUFingerState::ready;
            }
            break;

        case NYUFingerState::ready:
            if (calibrate_request_)
            {
                calibrate_request_ = false;
                state_ = NYUFingerState::calibrate;
                robot_->joints->SetZeroCommands();
            }
            robot_->SendCommand();
            break;

        case NYUFingerState::calibrate:
            if (calib_ctrl_->Run())
            {
                state_ = NYUFingerState::ready;
            }
            robot_->SendCommand();
            break;
    }
}

bool NYUFinger::calibrate(const Vector3d& home_offset_rad)
{
    printf("NYUFinger::request_calibration called\n");
    Eigen::VectorXd hor = home_offset_rad;
    calib_ctrl_->UpdatePositionOffsets(hor);
    calibrate_request_ = true;
    return true;
}

}  // namespace nyu_finger
