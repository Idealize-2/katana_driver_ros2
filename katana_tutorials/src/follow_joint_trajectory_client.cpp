// =============================================================================
// Tutorial 1: follow_joint_trajectory_client.cpp
// =============================================================================
// ROS 2 port of the original follow_joint_trajectory_client.
//
// WHAT IT DOES:
//   Sends the Katana arm through a hardcoded 2-waypoint trajectory:
//     1. Calibration position  (arm at homed position)
//     2. Ready pose            (arm raised to neutral working position)
//
// HOW IT WORKS:
//   - Reads current joint state from /joint_states (via KatanaArmClient)
//   - Prints current and target positions for inspection
//   - Asks for confirmation before each move (press Enter / Ctrl+C to abort)
//   - Sends trajectory to /arm_controller/follow_joint_trajectory
//
// RUN:
//   ros2 run katana_tutorials follow_joint_trajectory_client
// =============================================================================

#include <cstdio>
#include <iomanip>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "katana_tutorials/katana_arm_client.hpp"

// Print a side-by-side comparison of current vs target positions.
static void previewMove(
  const std::string & label,
  const std::vector<double> & current,
  const std::vector<double> & target)
{
  const auto & names = katana_tutorials::KatanaArmClient::ARM_JOINT_NAMES;
  std::cout << "\n=== Preview: " << label << " ===\n";
  std::cout << std::left
            << std::setw(36) << "Joint"
            << std::setw(12) << "Current"
            << std::setw(12) << "Target"
            << "Delta\n";
  std::cout << std::string(66, '-') << "\n";
  for (std::size_t i = 0; i < names.size(); ++i) {
    double cur = (i < current.size()) ? current[i] : 0.0;
    double tgt = (i < target.size())  ? target[i]  : 0.0;
    std::cout << std::setw(36) << names[i]
              << std::setw(12) << std::fixed << std::setprecision(3) << cur
              << std::setw(12) << tgt
              << (tgt - cur > 0 ? "+" : "") << std::setprecision(3) << (tgt - cur)
              << " rad\n";
  }
  std::cout << std::string(66, '-') << "\n";
  std::cout << "Press Enter to execute, Ctrl+C to abort: " << std::flush;
  std::string line;
  std::getline(std::cin, line);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto arm = std::make_shared<katana_tutorials::KatanaArmClient>(
    "follow_joint_trajectory_client");

  // ── Wait for the arm's current joint state ─────────────────────────────
  if (!arm->waitForJointState(10.0)) {
    RCLCPP_FATAL(arm->get_logger(), "Could not get joint state. Is the arm running?");
    rclcpp::shutdown();
    return 1;
  }

  auto current = arm->currentJointPositions();

  // ── Define hardcoded waypoints (radians) ──────────────────────────────
  //  These values are for the Katana 400 6M180 after calibration.
  //  motor1=pan, motor2=lift, motor3=lift, motor4=lift, motor5=wrist_roll

  //  "Calibration pose" — arm at its homed/calibrated position
  std::vector<double> calibration_pos = {-3.03, 2.17, -2.22, -2.03, -2.99};

  //  "Ready pose" — arm raised to a neutral working position
  std::vector<double> ready_pos = {0.0, 1.57, 0.0, 0.0, 0.0};

  // ── Preview + confirm: move to calibration pose ─────────────────────────
  previewMove("Calibration pose", current, calibration_pos);

  auto traj1 = katana_tutorials::makeTrajectory(
    katana_tutorials::KatanaArmClient::ARM_JOINT_NAMES,
    current, calibration_pos, 5.0, 6.0);

  if (!arm->sendTrajectoryAndWait(traj1)) {
    RCLCPP_ERROR(arm->get_logger(), "Failed to reach calibration pose.");
    rclcpp::shutdown();
    return 1;
  }

  // ── Preview + confirm: move to ready pose ──────────────────────────────
  auto current2 = arm->currentJointPositions();
  previewMove("Ready pose", current2, ready_pos);

  auto traj2 = katana_tutorials::makeTrajectory(
    katana_tutorials::KatanaArmClient::ARM_JOINT_NAMES,
    current2, ready_pos, 5.0, 6.0);

  if (!arm->sendTrajectoryAndWait(traj2)) {
    RCLCPP_ERROR(arm->get_logger(), "Failed to reach ready pose.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(arm->get_logger(), "Tutorial complete!");
  rclcpp::shutdown();
  return 0;
}
