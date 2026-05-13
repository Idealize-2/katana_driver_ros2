// Copyright 2024 Kay (Katana ROS 2 Port)
// SPDX-License-Identifier: BSD-3-Clause
//
// katana_hardware_interface.hpp
// ros2_control hardware interface plugin for the Neuronics Katana 450 arm.
// This class bridges between the KNI SDK (CKatana / CLMBase) and the
// ros2_control SystemInterface so that MoveIt 2 can drive the physical arm.

#pragma once

#include <memory>
#include <string>
#include <vector>

// ros2_control
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// KNI SDK
#include "kniBase.h"   // pulls in CLMBase, CCdlSocket, CCplSerialCRC, etc.

namespace katana_driver
{

class KatanaHardwareInterface : public hardware_interface::SystemInterface
{
public:
  // -------------------------------------------------------------------------
  // Lifecycle hooks (called by controller_manager)
  // -------------------------------------------------------------------------

  /// Parse URDF hardware params; allocate state/command vectors.
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /// Export state interfaces (position, velocity) for each joint.
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /// Export command interfaces (position) for each joint.
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /// Open TCP socket → init protocol → load config → calibrate arm.
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Freeze motors and close the connection.
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Read encoder values from arm → convert to radians → fill hw_states_.
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /// Take command radians → convert to encoder ticks → send to arm.
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // -------------------------------------------------------------------------
  // Conversion helpers
  // -------------------------------------------------------------------------

  /// Convert encoder ticks to radians for joint [idx].
  double encoderToRad(int joint_idx, int encoder) const;

  /// Convert radians to encoder ticks for joint [idx].
  int radToEncoder(int joint_idx, double rad) const;

  // -------------------------------------------------------------------------
  // KNI objects
  // -------------------------------------------------------------------------
  std::unique_ptr<CCdlSocket>      device_;
  std::unique_ptr<CCplSerialCRC>   protocol_;
  std::unique_ptr<CLMBase>         katana_;

  // -------------------------------------------------------------------------
  // Config loaded from URDF <param> tags
  // -------------------------------------------------------------------------
  std::string ip_address_;      // e.g. "192.168.1.1"
  int         tcp_port_{5566};  // Katana always listens on 5566
  std::string config_file_;     // path to katana6M180_F.cfg (or similar)
  bool        calibrate_on_startup_{true};

  // -------------------------------------------------------------------------
  // Number of joints managed by this interface
  // -------------------------------------------------------------------------
  static constexpr std::size_t NUM_JOINTS = 7;   // 5 arm + 2 finger

  // -------------------------------------------------------------------------
  // State / command double arrays exposed to ros2_control
  // -------------------------------------------------------------------------
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_commands_positions_;

  // -------------------------------------------------------------------------
  // Per-joint encoder calibration data (populated after katana_->create())
  // -------------------------------------------------------------------------
  struct JointEncoderInfo {
    int    enc_per_cycle;   // encoder ticks per 360°
    double angle_offset;    // radians (loaded from .cfg)
    int    direction;       // +1 or -1 (rotation direction)
    int    enc_min;
    int    enc_max;
  };
  std::vector<JointEncoderInfo> joint_info_;

  rclcpp::Logger logger_{rclcpp::get_logger("KatanaHardwareInterface")};
};

}  // namespace katana_driver
