/**
 * \file dgm_nyu_finger.cpp
 * \brief The hardware wrapper of the NYUFinger robot
 * \author Julian Viereck, Huaijiang Zhu
 * \date June 15, 2021
 *
 * This file defines the DGMNYUFinger class.
 */

#include "nyu_finger/dynamic_graph_manager/dgm_nyu_finger.hpp"
#include "dynamic_graph_manager/ros.hpp"

namespace nyu_finger
{
DGMNYUFinger::DGMNYUFinger()
{
    was_in_safety_mode_ = false;
}

DGMNYUFinger::~DGMNYUFinger()
{
}

void DGMNYUFinger::initialize_hardware_communication_process()
{
    /**
     * Load the calibration parameters.
     */
    nyu_finger::Vector3d joint_index_to_zero;
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
            std::bind(&DGMNYUFinger::calibrate_joint_position_callback,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2)));

    std::string network_id;
    YAML::ReadParameter(
        params_["hardware_communication"], "network_id", network_id);

    nyu_finger_.initialize(network_id, serial_port);
}

bool DGMNYUFinger::is_in_safety_mode()
{
    // Check if any card is in an error state.
    if (nyu_finger_.has_error()) {
      was_in_safety_mode_ = true;
      static int counter = 0;
      if (counter % 2000 == 0) {
        printf("DGMSolo12: Going into safe mode as error was reported.\n");
      }
      counter += 1;
    }

    if (was_in_safety_mode_ || DynamicGraphManager::is_in_safety_mode())
    {
      static int counter = 0;
      was_in_safety_mode_ = true;
      if (counter % 2000 == 0)
      {
        printf("DGMSolo12: is_in_safety_mode.\n");
      }
      counter++;
    }
    return was_in_safety_mode_;
  }

  void DGMNYUFinger::compute_safety_controls()
  {
    // Check if there is an error with the motors. If so, best we can do is
    // to command zero torques.
    if (nyu_finger_.has_error()) {
      for (auto ctrl = motor_controls_map_.begin();
          ctrl != motor_controls_map_.end();
          ++ctrl)
      {
          ctrl->second.fill(0.0);
      }
    } else {
      // The motors are fine.
      // --> Run a D controller to damp the current motion.
      motor_controls_map_.at("ctrl_joint_torques") =
          -0.05 * sensors_map_.at("joint_velocities");
    }
  }

void DGMNYUFinger::get_sensors_to_map(dynamic_graph_manager::VectorDGMap& map)
{
    nyu_finger_.acquire_sensors();

    /**
     * Joint data.
     */
    map.at("joint_positions") = nyu_finger_.get_joint_positions();
    map.at("joint_velocities") = nyu_finger_.get_joint_velocities();
    map.at("joint_torques") = nyu_finger_.get_joint_torques();
    map.at("joint_target_torques") = nyu_finger_.get_joint_target_torques();
    map.at("joint_encoder_index") = nyu_finger_.get_joint_encoder_index();
    /**
     * Robot status.
     */
    dynamicgraph::Vector& map_motor_enabled = map.at("motor_enabled");
    dynamicgraph::Vector& map_motor_ready = map.at("motor_ready");
    dynamicgraph::Vector& map_motor_board_enabled =
        map.at("motor_board_enabled");
    dynamicgraph::Vector& map_motor_board_errors = map.at("motor_board_errors");
    const std::array<bool, 3>& motor_enabled = nyu_finger_.get_motor_enabled();
    const std::array<bool, 3>& motor_ready = nyu_finger_.get_motor_ready();
    const std::array<bool, 2>& motor_board_enabled =
        nyu_finger_.get_motor_board_enabled();
    const std::array<int, 2>& motor_board_errors =
        nyu_finger_.get_motor_board_errors();

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

void DGMNYUFinger::set_motor_controls_from_map(
    const dynamic_graph_manager::VectorDGMap& map)
{
    try
    {
        // Here we need to perform and internal copy. Otherwise the compilator
        // complains.
        ctrl_joint_torques_ = map.at("ctrl_joint_torques");
        // Actually send the control to the robot.
        nyu_finger_.send_target_joint_torque(ctrl_joint_torques_);
    }
    catch (const std::exception& e)
    {
        rt_printf(
            "DGMNYUFinger::set_motor_controls_from_map: "
            "Error sending controls, %s\n",
            e.what());
    }
}

void DGMNYUFinger::calibrate_joint_position_callback(
    mim_msgs::srv::JointCalibration::Request::SharedPtr,
    mim_msgs::srv::JointCalibration::Response::SharedPtr res)
{
    // Parse and register the command for further call.
    add_user_command(std::bind(&DGMNYUFinger::calibrate_joint_position,
                               this,
                               zero_to_index_angle_from_file_));

    // Return a sanity check that assert that the function has been correctly
    // registered in the hardware process.
    res->sanity_check = true;
}

void DGMNYUFinger::calibrate_joint_position(
    const nyu_finger::Vector3d& zero_to_index_angle)
{
    nyu_finger_.request_calibration(zero_to_index_angle);
}

}  // namespace solo