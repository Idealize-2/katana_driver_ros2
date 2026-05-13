// =============================================================================
// Tutorial 1: follow_joint_trajectory_client.cpp
// =============================================================================
// ROS 2 port of the original follow_joint_trajectory_client.
//
// WHAT IT DOES:
//   Sends the Katana arm through a hardcoded 2-waypoint trajectory:
//     1. Calibration position  (arm lowered / tucked)
//     2. Straight-up position  (arm pointing vertically)
//
// HOW IT WORKS (new ROS 2 approach):
//   - Reads current joint state from /joint_states (via KatanaArmClient)
//   - Builds a JointTrajectory message
//   - Sends it to /katana_arm_controller/follow_joint_trajectory (Action)
//   - Waits for the arm to finish moving
//
// RUN:
//   ros2 run katana_tutorials follow_joint_trajectory_client
// =============================================================================

#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "katana_tutorials/katana_arm_client.hpp"

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
  //  These values work for the Katana 450 6M 180 after calibration.
  //  motor1=pan, motor2=lift, motor3=lift, motor4=lift, motor5=wrist_roll

  //  "Calibration pose" — arm down, motors at their calibrated home
  std::vector<double> calibration_pos = {-2.96, 2.14, -2.16, -1.97, -2.93};

  //  "Straight up" — arm pointing vertically
  std::vector<double> straight_up_pos = {0.0, 1.57, 0.0, 0.0, 0.0};

  // ── Move to calibration pose ────────────────────────────────────────────
  RCLCPP_INFO(arm->get_logger(), "Moving to calibration pose...");
  auto traj1 = katana_tutorials::makeTrajectory(
    katana_tutorials::KatanaArmClient::ARM_JOINT_NAMES,
    current, calibration_pos, 5.0, 6.0);

  if (!arm->sendTrajectoryAndWait(traj1)) {
    RCLCPP_ERROR(arm->get_logger(), "Failed to reach calibration pose.");
    rclcpp::shutdown();
    return 1;
  }

  // ── Move to straight-up ─────────────────────────────────────────────────
  RCLCPP_INFO(arm->get_logger(), "Moving to straight-up position...");
  auto current2 = arm->currentJointPositions();
  auto traj2    = katana_tutorials::makeTrajectory(
    katana_tutorials::KatanaArmClient::ARM_JOINT_NAMES,
    current2, straight_up_pos, 5.0, 6.0);

  if (!arm->sendTrajectoryAndWait(traj2)) {
    RCLCPP_ERROR(arm->get_logger(), "Failed to reach straight-up position.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(arm->get_logger(), "Tutorial complete!");
  rclcpp::shutdown();
  return 0;
}
