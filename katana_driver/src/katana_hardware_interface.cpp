// Copyright 2024 Kay (Katana ROS 2 Port)
// SPDX-License-Identifier: BSD-3-Clause
//
// katana_hardware_interface.cpp

#include "katana_driver/katana_hardware_interface.hpp"

#include <cmath>
#include <stdexcept>
#include <algorithm>

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
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Read Parameters ---
  try {
    // Determine connection type (default to tcp if not specified)
    connection_type_ = info_.hardware_parameters.count("connection_type") ? 
                       info_.hardware_parameters.at("connection_type") : "tcp";
    
    config_file_ = info_.hardware_parameters.at("config_file");

    if (connection_type_ == "tcp") {
      ip_address_ = info_.hardware_parameters.at("ip_address");
      tcp_port_   = info_.hardware_parameters.count("tcp_port") ? 
                    std::stoi(info_.hardware_parameters.at("tcp_port")) : 5566;
    } else if (connection_type_ == "serial") {
      serial_port_number_ = info_.hardware_parameters.count("serial_port") ? 
                            std::stoi(info_.hardware_parameters.at("serial_port")) : 0;
      serial_baud_        = info_.hardware_parameters.count("serial_baud") ? 
                            std::stoi(info_.hardware_parameters.at("serial_baud")) : 57600;
    } else {
      RCLCPP_FATAL(logger_, "Invalid connection_type: %s. Must be 'tcp' or 'serial'.", connection_type_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.hardware_parameters.count("calibrate_on_startup")) {
      calibrate_on_startup_ = (info_.hardware_parameters.at("calibrate_on_startup") == "true");
    } else {
      calibrate_on_startup_ = true;
    }

  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Error parsing hardware parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (connection_type_ == "tcp") {
    RCLCPP_INFO(logger_, "Katana HW Interface — TCP: %s:%d  cfg: %s", ip_address_.c_str(), tcp_port_, config_file_.c_str());
  } else {
    RCLCPP_INFO(logger_, "Katana HW Interface — Serial: /dev/ttyS%d (%d baud)  cfg: %s", serial_port_number_, serial_baud_, config_file_.c_str());
  }

  // Allocate vectors
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
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]);
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
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]);
  }
  return command_interfaces;
}

// ---------------------------------------------------------------------------
// on_activate
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KatanaHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating Katana hardware interface...");

  try {
    // 1. Open Device
    if (connection_type_ == "tcp") {
      device_ = std::make_unique<CCdlSocket>(const_cast<char*>(ip_address_.c_str()), tcp_port_);
      RCLCPP_INFO(logger_, "TCP socket opened to %s:%d", ip_address_.c_str(), tcp_port_);
    } else {
      TCdlCOMDesc ccd;
      ccd.port = serial_port_number_;
      ccd.baud = serial_baud_;
      ccd.data = 8;
      ccd.parity = 'N';
      ccd.stop = 1;
      ccd.rttc = 100; // read timeout
      ccd.wttc = 100; // write timeout
      
      device_ = std::make_unique<CCdlCOM>(ccd);
      RCLCPP_INFO(logger_, "Serial port /dev/ttyS%d opened at %d baud", serial_port_number_, serial_baud_);
    }

    // 2. Init Protocol
    protocol_ = std::make_unique<CCplSerialCRC>();
    protocol_->init(device_.get());

    // 3. Create Katana Arm
    katana_ = std::make_unique<CLMBase>();
    katana_->create(config_file_.c_str(), protocol_.get());
    RCLCPP_INFO(logger_, "Katana arm object created.");

    // 4. Cache joint info
    const TKatMOT * motors = katana_->GetBase()->GetMOT();
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      if (static_cast<int>(i) >= motors->cnt) continue;
      
      const TMotInit * init = motors->arr[i].GetInitialParameters();
      joint_info_[i].enc_per_cycle = init->encodersPerCycle;
      joint_info_[i].angle_offset  = init->angleOffset;
      joint_info_[i].direction      = init->rotationDirection;
      joint_info_[i].enc_min        = motors->arr[i].GetEncoderMinPos();
      joint_info_[i].enc_max        = motors->arr[i].GetEncoderMaxPos();
    }

    // 5. Calibrate
    if (calibrate_on_startup_) {
      RCLCPP_INFO(logger_, "Calibrating Katana arm...");
      katana_->calibrate();
    }

    katana_->setRobotVelocityLimit(20);

    // Initial read
    std::vector<int> encoders = katana_->getRobotEncoders(true);
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      if (i < encoders.size()) {
        hw_states_positions_[i] = encoderToRad(static_cast<int>(i), encoders[i]);
      }
      hw_commands_positions_[i] = hw_states_positions_[i];
    }

  } catch (const Exception & e) {
    RCLCPP_FATAL(logger_, "KNI exception: %s", e.message().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Exception: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KatanaHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (katana_) {
    katana_->freezeRobot();
    katana_->switchRobotOff();
  }
  katana_.reset();
  protocol_.reset();
  device_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// read
// ---------------------------------------------------------------------------
hardware_interface::return_type KatanaHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!katana_) return hardware_interface::return_type::ERROR;
  try {
    std::vector<int> encoders = katana_->getRobotEncoders(true);
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      if (i < encoders.size()) {
        hw_states_positions_[i] = encoderToRad(static_cast<int>(i), encoders[i]);
      }
    }
  } catch (...) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write
// ---------------------------------------------------------------------------
hardware_interface::return_type KatanaHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!katana_) return hardware_interface::return_type::ERROR;
  try {
    const TKatMOT * motors = katana_->GetBase()->GetMOT();
    std::vector<int> targets(static_cast<std::size_t>(motors->cnt), 0);
    for (int i = 0; i < motors->cnt; ++i) {
      int enc = radToEncoder(i, hw_commands_positions_[i]);
      enc = std::max(joint_info_[i].enc_min, std::min(joint_info_[i].enc_max, enc));
      targets[i] = enc;
    }
    katana_->moveRobotToEnc(targets, false, 100);
  } catch (...) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

double KatanaHardwareInterface::encoderToRad(int joint_idx, int encoder) const
{
  const auto & ji = joint_info_[joint_idx];
  if (ji.enc_per_cycle == 0) return 0.0;
  return ji.angle_offset + ji.direction * (static_cast<double>(encoder) / ji.enc_per_cycle) * (2.0 * M_PI);
}

int KatanaHardwareInterface::radToEncoder(int joint_idx, double rad) const
{
  const auto & ji = joint_info_[joint_idx];
  if (ji.enc_per_cycle == 0) return 0;
  double enc_f = (rad - ji.angle_offset) / (ji.direction * (2.0 * M_PI) / ji.enc_per_cycle);
  return static_cast<int>(std::round(enc_f));
}

} // namespace katana_driver
