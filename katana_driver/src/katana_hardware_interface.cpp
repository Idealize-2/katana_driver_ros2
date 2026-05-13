// Copyright 2024 Kay (Katana ROS 2 Port)
// SPDX-License-Identifier: BSD-3-Clause
//
// katana_hardware_interface.cpp
// Implementation of the ros2_control SystemInterface for the Katana 450 arm.
//
// Data flow:
//   [MoveIt 2] --cmd_pos (rad)--> write() --encoder ticks--> [KNI / Arm]
//   [KNI / Arm] --encoder ticks--> read()  --rad--> [MoveIt 2 / /joint_states]

#include "katana_driver/katana_hardware_interface.hpp"

#include <cmath>
#include <stdexcept>

// pluginlib macro — registers the class so controller_manager can dlopen it
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  katana_driver::KatanaHardwareInterface,
  hardware_interface::SystemInterface)

namespace katana_driver
{

// ---------------------------------------------------------------------------
// on_init
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KatanaHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Call base-class init first (validates joint/interface counts from URDF)
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Read parameters from the <hardware> block in the .ros2_control.xacro ---
  try {
    ip_address_  = info_.hardware_parameters.at("ip_address");
    config_file_ = info_.hardware_parameters.at("config_file");
  } catch (const std::out_of_range &) {
    RCLCPP_FATAL(logger_,
      "Missing required hardware parameters: 'ip_address' and/or 'config_file'.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("tcp_port")) {
    tcp_port_ = std::stoi(info_.hardware_parameters.at("tcp_port"));
  }
  if (info_.hardware_parameters.count("calibrate_on_startup")) {
    calibrate_on_startup_ =
      (info_.hardware_parameters.at("calibrate_on_startup") == "true");
  }

  RCLCPP_INFO(logger_, "Katana HW Interface — ip: %s  port: %d  cfg: %s",
    ip_address_.c_str(), tcp_port_, config_file_.c_str());

  // Allocate state / command vectors (one slot per joint declared in URDF)
  hw_states_positions_.assign(info_.joints.size(), 0.0);
  hw_states_velocities_.assign(info_.joints.size(), 0.0);
  hw_commands_positions_.assign(info_.joints.size(), 0.0);
  joint_info_.resize(info_.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// export_state_interfaces
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
KatanaHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_states_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_states_velocities_[i]);
  }
  return state_interfaces;
}

// ---------------------------------------------------------------------------
// export_command_interfaces
// ---------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
KatanaHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_positions_[i]);
  }
  return command_interfaces;
}

// ---------------------------------------------------------------------------
// on_activate  — open connection, init KNI, calibrate
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KatanaHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating Katana hardware interface...");

  try {
    // 1. Open TCP socket to the arm's Ethernet controller
    //    CCdlSocket takes char* (old KNI API) so we cast away const safely
    device_ = std::make_unique<CCdlSocket>(
      const_cast<char*>(ip_address_.c_str()), tcp_port_);
    RCLCPP_INFO(logger_, "TCP socket opened to %s:%d", ip_address_.c_str(), tcp_port_);

    // 2. Init the Serial-CRC protocol layer
    protocol_ = std::make_unique<CCplSerialCRC>();
    protocol_->init(device_.get());
    RCLCPP_INFO(logger_, "KNI protocol initialised.");

    // 3. Create the high-level CLMBase object and load the .cfg file
    katana_ = std::make_unique<CLMBase>();
    katana_->create(config_file_.c_str(), protocol_.get());
    RCLCPP_INFO(logger_, "Katana arm object created from config: %s", config_file_.c_str());

    // 4. Cache encoder-to-radian conversion data for each joint
    const TKatMOT * motors = katana_->GetBase()->GetMOT();
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      if (static_cast<int>(i) >= motors->cnt) {
        // Extra joints (e.g. right finger mirrors left)
        joint_info_[i] = joint_info_[i - 1];
        continue;
      }
      const TMotInit * init = motors->arr[i].GetInitialParameters();
      joint_info_[i].enc_per_cycle = init->encodersPerCycle;
      joint_info_[i].angle_offset  = init->angleOffset;   // radians
      joint_info_[i].direction      = init->rotationDirection;
      joint_info_[i].enc_min        = motors->arr[i].GetEncoderMinPos();
      joint_info_[i].enc_max        = motors->arr[i].GetEncoderMaxPos();

      RCLCPP_DEBUG(logger_,
        "Joint[%zu] enc_per_cycle=%d  angle_offset=%.4f  dir=%d  min=%d  max=%d",
        i,
        joint_info_[i].enc_per_cycle,
        joint_info_[i].angle_offset,
        joint_info_[i].direction,
        joint_info_[i].enc_min,
        joint_info_[i].enc_max);
    }

    // 5. Calibrate (moves arm to limit switches to find zero)
    if (calibrate_on_startup_) {
      RCLCPP_INFO(logger_, "Calibrating Katana arm — do NOT obstruct the workspace!");
      katana_->calibrate();
      RCLCPP_INFO(logger_, "Calibration complete.");
    }

    // 6. Set a safe default velocity limit (encoders / 10ms)
    katana_->setRobotVelocityLimit(20);

    // 7. Do one read to initialise hw_states_ with the real current position
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      // Motor index into KNI is 0-based; gripper fingers are motor 5/6
      int motor_idx = static_cast<int>(std::min(i, info_.joints.size() - 1));
      if (motor_idx < motors->cnt) {
        int enc = katana_->getMotorEncoders(static_cast<short>(motor_idx), true);
        hw_states_positions_[i] = encoderToRad(static_cast<int>(i), enc);
      }
      hw_commands_positions_[i] = hw_states_positions_[i];
    }

  } catch (const Exception & e) {
    RCLCPP_FATAL(logger_, "KNI exception during activation: %s", e.message().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Exception during activation: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Katana hardware interface activated successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate  — freeze motors, release connection
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KatanaHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating Katana hardware interface...");
  try {
    if (katana_) {
      katana_->freezeRobot();
      katana_->switchRobotOff();
    }
  } catch (const Exception & e) {
    RCLCPP_WARN(logger_, "KNI exception during deactivation: %s", e.message().c_str());
  }
  katana_.reset();
  protocol_.reset();
  device_.reset();
  RCLCPP_INFO(logger_, "Katana hardware interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// read  — fetch encoder values, convert to radians
// ---------------------------------------------------------------------------
hardware_interface::return_type KatanaHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!katana_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    // Bulk-read all motor positions in one serial round-trip
    std::vector<int> encoders = katana_->getRobotEncoders(true /*refresh*/);

    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      if (i < encoders.size()) {
        hw_states_positions_[i] = encoderToRad(static_cast<int>(i), encoders[i]);
      }
      // Velocity is not directly readable from KNI; report zero for now.
      hw_states_velocities_[i] = 0.0;
    }
  } catch (const Exception & e) {
    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
      "KNI read error: %s", e.message().c_str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write  — convert command radians → encoder ticks → send to arm
// ---------------------------------------------------------------------------
hardware_interface::return_type KatanaHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!katana_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    // Build a vector of target encoder values (one per motor KNI knows about)
    const TKatMOT * motors = katana_->GetBase()->GetMOT();
    std::vector<int> target_encoders(static_cast<std::size_t>(motors->cnt), 0);

    for (int i = 0; i < motors->cnt; ++i) {
      if (static_cast<std::size_t>(i) < hw_commands_positions_.size()) {
        int enc = radToEncoder(i, hw_commands_positions_[i]);

        // Clamp to the motor's safe range
        enc = std::max(joint_info_[i].enc_min, std::min(joint_info_[i].enc_max, enc));
        target_encoders[i] = enc;
      }
    }

    // Send all joint targets simultaneously (non-blocking)
    // waitUntilReached=false → controller_manager calls write() again next cycle
    katana_->moveRobotToEnc(target_encoders, false /*waitUntilReached*/, 100 /*tolerance*/);

  } catch (const Exception & e) {
    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000,
      "KNI write error: %s", e.message().c_str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Private helpers — encoder <-> radian conversion
// ---------------------------------------------------------------------------

// Formula (from KNI source / katana_arm_kinematics package):
//   angle_rad = angle_offset + direction * (encoder / enc_per_cycle) * 2*PI
//   encoder   = (angle_rad - angle_offset) / (direction * 2*PI / enc_per_cycle)

double KatanaHardwareInterface::encoderToRad(int joint_idx, int encoder) const
{
  if (joint_idx < 0 || static_cast<std::size_t>(joint_idx) >= joint_info_.size()) {
    return 0.0;
  }
  const auto & ji = joint_info_[joint_idx];
  if (ji.enc_per_cycle == 0) return 0.0;

  return ji.angle_offset +
         ji.direction * (static_cast<double>(encoder) / ji.enc_per_cycle) *
         (2.0 * M_PI);
}

int KatanaHardwareInterface::radToEncoder(int joint_idx, double rad) const
{
  if (joint_idx < 0 || static_cast<std::size_t>(joint_idx) >= joint_info_.size()) {
    return 0;
  }
  const auto & ji = joint_info_[joint_idx];
  if (ji.enc_per_cycle == 0) return 0;

  double enc_f = (rad - ji.angle_offset) /
                 (ji.direction * (2.0 * M_PI) / ji.enc_per_cycle);
  return static_cast<int>(std::round(enc_f));
}

}  // namespace katana_driver
