// Copyright 2024 Kay (Katana ROS 2 Port)
// SPDX-License-Identifier: BSD-3-Clause
//
// katana_hardware_interface.hpp
// ros2_control hardware interface plugin for the Neuronics Katana 450 arm.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ros2_control
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/set_bool.hpp"

// KNI SDK
#include "kniBase.h"       // CLMBase
#include "KNI/cdlSocket.h" // CCdlSocket
#include "KNI/cdlCOM.h"    // CCdlCOM (Serial)
#include "KNI/cplSerial.h" // CCplSerialCRC

namespace katana_driver
{

class KatanaHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KatanaHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Parameters ────────────────────────────────────────────────────────────
  std::string connection_type_;     // "tcp" or "serial"
  
  // TCP Params
  std::string ip_address_;
  int         tcp_port_;
  
  // Serial Params
  int         serial_port_number_; // 0 for /dev/ttyS0, 1 for /dev/ttyS1, etc.
  int         serial_baud_;
  
  std::string config_file_;
  bool        calibrate_on_startup_;

  // ── KNI Objects ───────────────────────────────────────────────────────────
  // We use CCdlBase as the abstract device to support both Socket and COM
  std::unique_ptr<CCdlBase>       device_;
  std::unique_ptr<CCplSerialCRC>  protocol_;
  std::unique_ptr<CLMBase>        katana_;

  // ── Hardware State/Commands ───────────────────────────────────────────────
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_commands_positions_;

  static constexpr double kDeadbandRad = 0.009;  // ~0.5 deg

  // ── Async KNI worker ──────────────────────────────────────────────────────
  std::mutex               kni_mtx_;
  std::vector<double>      hw_pos_cache_;    // KNI thread → read()
  std::vector<double>      hw_cmd_cache_;    // write() → KNI thread
  std::atomic<bool>        kni_running_{false};
  std::thread              kni_thread_;


  // KNI-thread-only state (never touched by the CM thread)
  std::vector<int>         kni_last_enc_;
  std::vector<double>      kni_last_vel_;
  std::vector<double>      kni_last_cmd_;
  int                      kni_idle_count_  = 0;
  bool                     kni_hold_sent_    = false;
  int                      kni_gripper_last_cmd_enc_ = -1;  // -1 = uninitialized, forces first send
  //change in urdf in moveit config ros2control
  int                      gripper_open_enc_ = 30770;
  int                      gripper_close_enc_= 15000;
  static constexpr int     kSplineT          = 50;  // 500 ms per segment — buffer over worst-case 410ms loop
  static constexpr int     kIdleThresh      = 3;   // stable cycles → moreflag=1
  static constexpr int     kEncMargin       = 200; // ticks from firmware limit
  static constexpr int     kGripperDeadband = 50;  // enc ticks — avoids re-triggering firmware ramp on unchanged target

  // Pre-allocated work buffers — avoids per-cycle heap allocation in kni_loop()
  std::vector<int>         kni_target_enc_;
  std::vector<double>      kni_ve_;

  // ── Motor power service ───────────────────────────────────────────────────
  rclcpp::Node::SharedPtr                                   svc_node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr        motor_power_svc_;
  rclcpp::executors::SingleThreadedExecutor                 svc_executor_;
  std::thread                                               svc_thread_;
  std::atomic<bool>                                         motors_powered_{true};
  std::atomic<bool>                                         reenable_requested_{false};

  // ── Internal Helpers ──────────────────────────────────────────────────────
  struct JointEncoderInfo {
    int    enc_per_cycle;
    double angle_offset;
    int    direction;
    int    enc_min;
    int    enc_max;
  };
  std::vector<JointEncoderInfo> joint_info_;
  std::vector<double>           urdf_offsets_;   // KNI angle at URDF joint-zero for each joint
  std::vector<double>           urdf_flips_;     // +1.0 or -1.0 per joint to match URDF axis direction

  double encoderToRad(int joint_idx, int encoder) const;
  int    radToEncoder(int joint_idx, double rad) const;
  void   kni_loop();

  rclcpp::Logger logger_{rclcpp::get_logger("KatanaHardwareInterface")};
};

}  // namespace katana_driver
