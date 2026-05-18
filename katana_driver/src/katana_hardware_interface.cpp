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
      std::string raw = info_.hardware_parameters.at("calibrate_on_startup");
      // Accept "true","True","TRUE","1","yes" as truthy.
      calibrate_on_startup_ = (raw == "true" || raw == "True" || raw == "TRUE"
                                || raw == "1"  || raw == "yes");
      RCLCPP_INFO(logger_, "calibrate_on_startup raw='%s' → %s",
                  raw.c_str(), calibrate_on_startup_ ? "WILL calibrate" : "SKIP calibrate");
    } else {
      calibrate_on_startup_ = true;
      RCLCPP_INFO(logger_, "calibrate_on_startup not set — defaulting to calibrate");
    }

  } catch (const Exception & e) {
    RCLCPP_FATAL(logger_, "KNI exception in on_init(): %s", e.message().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "std::exception in on_init(): %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_FATAL(logger_, "Unknown exception in on_init()");
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

  // Per-joint URDF offsets and direction flips.
  // offset: KNI angle when arm is at URDF joint-zero  → "urdf_offset_<joint_name>"
  // flip  : +1.0 or -1.0 to match URDF axis sign       → "urdf_flip_<joint_name>"
  urdf_offsets_.resize(info_.joints.size(), 0.0);
  urdf_flips_.resize(info_.joints.size(), 1.0);
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string & jname = info_.joints[i].name;
    if (info_.hardware_parameters.count("urdf_offset_" + jname)) {
      urdf_offsets_[i] = std::stod(info_.hardware_parameters.at("urdf_offset_" + jname));
    }
    if (info_.hardware_parameters.count("urdf_flip_" + jname)) {
      double f = std::stod(info_.hardware_parameters.at("urdf_flip_" + jname));
      urdf_flips_[i] = (f < 0.0) ? -1.0 : 1.0;
    }
    RCLCPP_INFO(logger_, "joint[%zu] %s  offset=%.4f  flip=%.0f",
                i, jname.c_str(), urdf_offsets_[i], urdf_flips_[i]);
  }

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

    // 5. Clear any motor fault flags left from a previous session.
    //    If the arm has an error flag set, moveRobotToEnc will immediately
    //    return KATANA_ERROR_FLAG and the first write cycle will fail.
    RCLCPP_INFO(logger_, "Clearing motor fault flags (unBlock)...");
    katana_->unBlock();

    // 6. Calibrate (moves joints to limits and back to find zero).
    //    Always required after power-up. Skip only if the arm is confirmed
    //    already calibrated from this power cycle (calibrate_on_startup=false).
    if (calibrate_on_startup_) {
      RCLCPP_INFO(logger_, "Calibrating Katana arm — arm will move to all joint limits...");
      katana_->calibrate();
      RCLCPP_INFO(logger_, "Calibration complete.");
      // Calibration drives joints into mechanical stops which sets error flags.
      // Clear them now so the first moveRobotToEnc call is not rejected.
      katana_->unBlock();
    } else {
      RCLCPP_WARN(logger_, "Skipping calibration (calibrate_on_startup=false). "
                           "Ensure the arm was calibrated in this power cycle or "
                           "moveRobotToEnc will throw 'Axis not calibrated'.");
    }

    // Initial read — seed hw_commands from actual encoder positions so the
    // first write cycle is a no-op (hold-in-place) rather than a jump to 0.
    std::vector<int> encoders = katana_->getRobotEncoders(true);
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      if (i < encoders.size()) {
        hw_states_positions_[i] = encoderToRad(static_cast<int>(i), encoders[i]);
      }
      hw_commands_positions_[i] = hw_states_positions_[i];
    }
    // Seed shared caches so read() returns a valid position immediately.
    hw_pos_cache_.resize(info_.joints.size());
    hw_cmd_cache_.resize(info_.joints.size());
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      hw_pos_cache_[i] = hw_states_positions_[i];
      hw_cmd_cache_[i] = hw_states_positions_[i];
    }

    // Seed KNI-thread state from actual encoder positions.
    {
      int mc = katana_->GetBase()->GetMOT()->cnt;
      kni_last_enc_.assign(mc, 0);
      kni_last_vel_.assign(mc, 0.0);
      for (int i = 0; i < mc && i < static_cast<int>(encoders.size()); ++i)
        kni_last_enc_[i] = encoders[i];
      kni_last_cmd_.assign(info_.joints.size(), 0.0);
      for (std::size_t i = 0; i < info_.joints.size(); ++i)
        kni_last_cmd_[i] = hw_states_positions_[i];
      kni_idle_count_ = kIdleThresh;  // first send will be moreflag=1 (hold)
      kni_target_enc_.assign(mc, 0);
      kni_ve_.assign(mc, 0.0);
    }

    // Launch background KNI worker.
    kni_running_ = true;
    kni_thread_  = std::thread([this]() { kni_loop(); });

    // Spin a tiny service node so the tester can disable/re-enable motor power.
    svc_node_ = std::make_shared<rclcpp::Node>("katana_hw_motor_power");
    motor_power_svc_ = svc_node_->create_service<std_srvs::srv::SetBool>(
      "katana_hw/set_motors_enabled",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr       resp)
      {
        if (req->data && !motors_powered_) {
          reenable_requested_ = true;   // handled in write() on the CM thread
        } else if (!req->data && motors_powered_) {
          try {
            katana_->switchRobotOff();
            motors_powered_ = false;
            RCLCPP_INFO(logger_, "Motors OFF — arm can be moved manually. Press 'e' to re-enable.");
          } catch (...) {}
        }
        resp->success = true;
        resp->message = req->data ? "enable requested" : "motors off";
      });
    svc_executor_.add_node(svc_node_);
    svc_thread_ = std::thread([this]() { svc_executor_.spin(); });

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
  svc_executor_.cancel();
  if (svc_thread_.joinable()) svc_thread_.join();
  svc_node_.reset();

  kni_running_ = false;
  if (kni_thread_.joinable()) kni_thread_.join();

  if (katana_) {
    try { katana_->freezeRobot(); } catch (...) {}
    try { katana_->switchRobotOff(); } catch (...) {}
  }
  katana_.reset();
  protocol_.reset();
  device_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// read  — instant mutex copy from KNI worker cache
// ---------------------------------------------------------------------------
hardware_interface::return_type KatanaHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!katana_) return hardware_interface::return_type::ERROR;
  std::lock_guard<std::mutex> lock(kni_mtx_);
  hw_states_positions_ = hw_pos_cache_;
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write  — instant mutex copy to KNI worker cache
// ---------------------------------------------------------------------------
hardware_interface::return_type KatanaHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!katana_) return hardware_interface::return_type::ERROR;
  std::lock_guard<std::mutex> lock(kni_mtx_);
  hw_cmd_cache_ = hw_commands_positions_;
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// kni_loop  — background thread: all actual KNI TCP communication runs here
// ---------------------------------------------------------------------------
void KatanaHardwareInterface::kni_loop()
{
  while (kni_running_) {
    try {
      // ── Reenable path ──────────────────────────────────────────────────────
      if (reenable_requested_) {
        reenable_requested_ = false;
        katana_->switchRobotOn();
        std::vector<int> enc = katana_->getRobotEncoders(true);
        std::vector<double> pos(info_.joints.size(), 0.0);
        for (std::size_t i = 0; i < info_.joints.size() && i < enc.size(); ++i)
          pos[i] = encoderToRad(static_cast<int>(i), enc[i]);
        {
          std::lock_guard<std::mutex> lk(kni_mtx_);
          hw_pos_cache_ = pos;
          hw_cmd_cache_ = pos;
        }
        for (std::size_t i = 0; i < kni_last_enc_.size() && i < enc.size(); ++i)
          kni_last_enc_[i] = enc[i];
        std::fill(kni_last_vel_.begin(), kni_last_vel_.end(), 0.0);
        kni_last_cmd_   = pos;
        kni_idle_count_ = kIdleThresh;
        kni_hold_sent_  = false;
        motors_powered_ = true;
        RCLCPP_INFO(logger_, "Motors ON — holding current position.");
        continue;
      }

      if (!motors_powered_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }

      // ── 1. Read actual encoders (~55 ms) ───────────────────────────────────
      std::vector<int> enc = katana_->getRobotEncoders(true);

      // ── 2+3. Update position cache and fetch latest command ──────────────────
      std::vector<double> cmds;
      {
        std::lock_guard<std::mutex> lk(kni_mtx_);
        for (std::size_t i = 0; i < info_.joints.size() && i < enc.size(); ++i)
          hw_pos_cache_[i] = encoderToRad(static_cast<int>(i), enc[i]);
        cmds = hw_cmd_cache_;
      }

      // ── 4. Convert to target encoders with safety margin ──────────────────
      const TKatMOT * motors = katana_->GetBase()->GetMOT();
      const int mc = motors->cnt;
      for (int i = 0; i < mc; ++i) {
        int e = radToEncoder(i, cmds[i]);
        e = std::max(joint_info_[i].enc_min + kEncMargin,
                     std::min(joint_info_[i].enc_max - kEncMargin, e));
        kni_target_enc_[i] = e;
      }

      // ── 5. Deadband / idle detection ───────────────────────────────────────
      bool changed = false;
      for (std::size_t i = 0; i < cmds.size(); ++i)
        if (std::abs(cmds[i] - kni_last_cmd_[i]) > kDeadbandRad)
          { changed = true; break; }

      if (changed) {
        kni_idle_count_ = 0;
        kni_hold_sent_  = false;  // new command → arm needs to move again
        kni_last_cmd_   = cmds;
      } else {
        kni_idle_count_++;
      }

      // Cache already updated in 2+3; if held at goal just skip the spline send.
      if (kni_hold_sent_) continue;

      // ── 6. Motor fault check (cached PVP) ─────────────────────────────────
      bool fault = false;
      for (int i = 0; i < mc; ++i) {
        short msf = motors->arr[i].GetPVP()->msf;
        if (msf == MSF_MOTCRASHED || msf == MSF_NOTVALID) {
          RCLCPP_WARN(logger_, "Motor %d fault (msf=%d) — skipping", i, (int)msf);
          fault = true;
          break;
        }
      }
      if (fault) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); continue; }

      // ── 7. moreflag: 0 = chain next segment, 1 = stop here ────────────────
      bool is_last  = (kni_idle_count_ >= kIdleThresh);
      int  moreflag = is_last ? 1 : 0;

      // ── 8. Hermite cubic coefficients → sendSplineToMotor (~245 ms total) ──
      const double T  = static_cast<double>(kSplineT);
      const double T2 = T * T, T3 = T * T * T;
      std::fill(kni_ve_.begin(), kni_ve_.end(), 0.0);
      if (!is_last) {
        for (int i = 0; i < mc; ++i)
          kni_ve_[i] = static_cast<double>(kni_target_enc_[i] - kni_last_enc_[i]) / T;
      }

      // Pre-update cache to goal BEFORE the 245ms send so JTC's hold command
      // is already at the goal position — closes the race window that caused
      // intermittent backward drift when JTC read a stale position mid-send.
      if (is_last) {
        std::lock_guard<std::mutex> lk(kni_mtx_);
        for (int i = 0; i < mc && i < static_cast<int>(hw_pos_cache_.size()); ++i)
          hw_pos_cache_[i] = encoderToRad(i, kni_target_enc_[i]);
      }

      for (int i = 0; i < mc; ++i) {
        const double s   = static_cast<double>(kni_last_enc_[i]);
        const double e   = static_cast<double>(kni_target_enc_[i]);
        const double vs  = kni_last_vel_[i];
        const double vei = kni_ve_[i];
        const double p1  = s;
        const double p2  = vs * T;
        const double p3  = 3.0*(e-s) - (2.0*vs + vei)*T;
        const double p4  = (vs + vei)*T - 2.0*(e-s);
        katana_->sendSplineToMotor(
          static_cast<short>(i),
          static_cast<short>(kni_target_enc_[i]),
          static_cast<short>(kSplineT),
          static_cast<short>(std::round(p1)),
          static_cast<short>(std::round(64.0    * p2 / T)),
          static_cast<short>(std::round(1024.0  * p3 / T2)),
          static_cast<short>(std::round(32768.0 * p4 / T3)));
      }
      katana_->startSplineMovement(true /*exactflag*/, moreflag);

      // ── 9. Update KNI-thread state for next iteration ─────────────────────
      kni_last_enc_ = kni_target_enc_;
      kni_last_vel_ = kni_ve_;
      if (is_last) kni_hold_sent_ = true;

    } catch (const Exception & e) {
      RCLCPP_WARN(logger_, "KNI worker: %s", e.message().c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "KNI worker: %s", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } catch (...) {
      RCLCPP_WARN(logger_, "KNI worker: unknown exception");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

double KatanaHardwareInterface::encoderToRad(int joint_idx, int encoder) const
{
  const auto & ji = joint_info_[joint_idx];
  if (ji.enc_per_cycle == 0) return 0.0;
  double kni_rad = ji.angle_offset + ji.direction * (static_cast<double>(encoder) / ji.enc_per_cycle) * (2.0 * M_PI);
  return (kni_rad - urdf_offsets_[joint_idx]) * urdf_flips_[joint_idx];
}

int KatanaHardwareInterface::radToEncoder(int joint_idx, double rad) const
{
  const auto & ji = joint_info_[joint_idx];
  if (ji.enc_per_cycle == 0) return 0;
  double kni_rad = (rad * urdf_flips_[joint_idx]) + urdf_offsets_[joint_idx];
  double enc_f = (kni_rad - ji.angle_offset) / (ji.direction * (2.0 * M_PI) / ji.enc_per_cycle);
  return static_cast<int>(std::round(enc_f));
}

} // namespace katana_driver
