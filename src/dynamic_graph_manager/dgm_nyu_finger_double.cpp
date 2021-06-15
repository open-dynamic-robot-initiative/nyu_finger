/**
 * \file dgm_nyu_finger.cpp
 * \brief The hardware wrapper of the NYUFinger robot
 * \author Julian Viereck, Huaijiang Zhu
 * \date June 15, 2021
 *
 * This file defines the DGMNYUFingerDouble class.
 */

#include "nyu_finger/dynamic_graph_manager/dgm_nyu_finger_double.hpp"
#include "dynamic_graph_manager/ros.hpp"

namespace nyu_finger
{
DGMNYUFingerDouble::DGMNYUFingerDouble()
{
    was_in_safety_mode_ = false;
}

DGMNYUFingerDouble::~DGMNYUFingerDouble()
{
}

void DGMNYUFingerDouble::initialize_hardware_communication_process()
{
    /**
     * Load the calibration parameters.
     */
    nyu_finger::Vector6d joint_index_to_zero;
    YAML::ReadParameter(params_["hardware_communication"]["calibration"],
                        "index_to_zero_angle",
                        zero_to_index_angle_from_file_);

    // Get the hardware communication ros node handle.
    dynamic_graph_manager::RosNodePtr ros_node_handle =
        dynamic_graph_manager::get_ros_node(
            dynamic_graph_manager::HWC_ROS_NODE_NAME);

    /** Initialize the user commands. */
    ros_user_commands_.push_back(
        ros_node_handle->create_service<mim_msgs::srv::JointCalibration>(
            "calibrate_joint_position",
            std::bind(&DGMNYUFingerDouble::calibrate_joint_position_callback,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2)));

    std::string network_id_0;
    std::string network_id_1;
    YAML::ReadParameter(
        params_["hardware_communication"], "network_id_0", network_id);
    YAML::ReadParameter(
        params_["hardware_communication"], "network_id_1", network_id);

    nyu_finger_0_.initialize(network_id_0);
    nyu_finger_1_.initialize(network_id_1);
}

bool DGMNYUFingerDouble::is_in_safety_mode()
{
    // Check if any card is in an error state.
    if (nyu_finger_0_.has_error() || nyu_finger_1_.has_error()) {
      was_in_safety_mode_ = true;
      static int counter = 0;
      if (counter % 2000 == 0) {
        printf("DGMNYUFingerDouble: Going into safe mode as error was reported.\n");
      }
      counter += 1;
    }

    if (was_in_safety_mode_ || DynamicGraphManager::is_in_safety_mode())
    {
      static int counter = 0;
      was_in_safety_mode_ = true;
      if (counter % 2000 == 0)
      {
        printf("DGMNYUFingerDouble: is_in_safety_mode.\n");
      }
      counter++;
    }
    return was_in_safety_mode_;
  }

  void DGMNYUFingerDouble::compute_safety_controls()
  {
    // Check if there is an error with the motors. If so, best we can do is
    // to command zero torques.
    if (nyu_finger_0_.has_error() || nyu_finger_1_.has_error()) {
      for (auto ctrl = motor_controls_map_.begin();
          ctrl != motor_controls_map_.end();
          ++ctrl)
      {
          ctrl->second.fill(0.0);
      }
    } else {
      // The motors are fine.
      // --> Run a D controller to damp the current motion.
      motor_controls_map_.at("finger0.ctrl_joint_torques") =
          -0.05 * sensors_map_.at("finger0.joint_velocities");
      motor_controls_map_.at("finger1.ctrl_joint_torques") =
          -0.05 * sensors_map_.at("finger1.joint_velocities");
    }
  }

void DGMNYUFingerDouble::get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map)
{
    nyu_finger_.acquire_sensors();

    /**
     * Joint data.
     */
    map.at("finger0.joint_positions") = nyu_finger_0_.get_joint_positions();
    map.at("finger0.joint_velocities") = nyu_finger_0_.get_joint_velocities();
    map.at("finger0.joint_torques") = nyu_finger_0_.get_joint_torques();
    map.at("finger0.joint_target_torques") = nyu_finger_0_.get_joint_target_torques();
    map.at("finger0.joint_encoder_index") = nyu_finger_0_.get_joint_encoder_index();

    map.at("finger1.joint_positions") = nyu_finger_1_.get_joint_positions();
    map.at("finger1.joint_velocities") = nyu_finger_1_.get_joint_velocities();
    map.at("finger1.joint_torques") = nyu_finger_1_.get_joint_torques();
    map.at("finger1.joint_target_torques") = nyu_finger_1_.get_joint_target_torques();
    map.at("finger1.joint_encoder_index") = nyu_finger_1_.get_joint_encoder_index();
    /**
     * Robot status.
     */
    dynamicgraph::Vector& map_motor_enabled = map.at("finger0.motor_enabled");
    dynamicgraph::Vector& map_motor_ready = map.at("finger0.motor_ready");
    dynamicgraph::Vector& map_motor_board_enabled =
        map.at("motor_board_enabled");
    dynamicgraph::Vector& map_motor_board_errors = map.at("finger0.motor_board_errors");
    const std::array<bool, 3>& motor_enabled = nyu_finger_0_.get_motor_enabled();
    const std::array<bool, 3>& motor_ready = nyu_finger_0_.get_motor_ready();
    const std::array<bool, 2>& motor_board_enabled =
        nyu_finger_0_.get_motor_board_enabled();
    const std::array<int, 2>& motor_board_errors =
        nyu_finger_0_.get_motor_board_errors();

    for (size_t i = 0; i < motor_enabled.size(); ++i)
    {
        map_motor_enabled[i] = motor_enabled[i];
        map_motor_ready[i] = motor_ready[i];
    }
    for (size_t i = 0; i < motor_board_enabled.size(); ++i)
    {
        map_motor_board_enabled[i] = motor_board_enabled[i];
        map_motor_board_errors[i] = motor_board_errors[i];
    }

    dynamicgraph::Vector& map_motor_enabled = map.at("finger1.motor_enabled");
    dynamicgraph::Vector& map_motor_ready = map.at("finger1.motor_ready");
    dynamicgraph::Vector& map_motor_board_enabled =
        map.at("motor_board_enabled");
    dynamicgraph::Vector& map_motor_board_errors = map.at("finger1.motor_board_errors");
    const std::array<bool, 3>& motor_enabled = nyu_finger_1_.get_motor_enabled();
    const std::array<bool, 3>& motor_ready = nyu_finger_1_.get_motor_ready();
    const std::array<bool, 2>& motor_board_enabled =
        nyu_finger_1_.get_motor_board_enabled();
    const std::array<int, 2>& motor_board_errors =
        nyu_finger_1_.get_motor_board_errors();

    for (size_t i = 0; i < motor_enabled.size(); ++i)
    {
        map_motor_enabled[i] = motor_enabled[i];
        map_motor_ready[i] = motor_ready[i];
    }
    for (size_t i = 0; i < motor_board_enabled.size(); ++i)
    {
        map_motor_board_enabled[i] = motor_board_enabled[i];
        map_motor_board_errors[i] = motor_board_errors[i];
    }
}

void DGMNYUFingerDouble::set_motor_controls_from_map(
    const dynamic_graph_manager::VectorDGMap& map)
{
    try
    {
        // Here we need to perform and internal copy. Otherwise the compilator
        // complains.
        ctrl_joint_torques_ = map.at("finger0.ctrl_joint_torques");
        // Actually send the control to the robot.
        nyu_finger_0_.send_target_joint_torque(ctrl_joint_torques_);

        ctrl_joint_torques_ = map.at("finger1.ctrl_joint_torques");
        // Actually send the control to the robot.
        nyu_finger_1_.send_target_joint_torque(ctrl_joint_torques_);
    }
    catch (const std::exception& e)
    {
        rt_printf(
            "DGMNYUFingerDouble::set_motor_controls_from_map: "
            "Error sending controls, %s\n",
            e.what());
    }
}

void DGMNYUFingerDouble::calibrate_joint_position_callback(
    mim_msgs::srv::JointCalibration::Request::SharedPtr,
    mim_msgs::srv::JointCalibration::Response::SharedPtr res)
{
    // Parse and register the command for further call.
    add_user_command(std::bind(&DGMNYUFingerDouble::calibrate_joint_position,
                               this,
                               zero_to_index_angle_from_file_));

    // Return a sanity check that assert that the function has been correctly
    // registered in the hardware process.
    res->sanity_check = true;
}

void DGMNYUFingerDouble::calibrate_joint_position(
    const nyu_finger::Vector6d& zero_to_index_angle)
{
    nyu_finger_0_.request_calibration(zero_to_index_angle.head<3>());
    nyu_finger_1_.request_calibration(zero_to_index_angle.tail<3>());
}

}  // namespace solo