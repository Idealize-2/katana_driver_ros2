// =============================================================================
// Tutorial 5: pr2_joint_trajectory_client.cpp
// =============================================================================
// ROS 2 — Multi-waypoint trajectory demo for the Katana 400 6M180 arm.
//
// WHAT IT DOES:
//   Sends the arm through a scripted 4-waypoint sequence using the
//   FollowJointTrajectory action server:
//
//     1. current         — wherever the arm is right now   (t =  0 s)
//     2. calibration     — post-calibration rest pose      (t =  5 s)
//     3. straight_up     — arm pointing vertically         (t = 10 s)
//     4. calibration     — back to rest pose               (t = 15 s)
//
//   Each segment is sent as a separate FollowJointTrajectory goal so the
//   arm reaches each waypoint before the next command is sent.
//
// REQUIRES:
//   ros2_control driver running (real_hardware.launch.py or full_system.launch.py)
//
// RUN:
//   ros2 run katana_tutorials pr2_joint_trajectory_client
// =============================================================================

#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "katana_tutorials/katana_arm_client.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto arm = std::make_shared<katana_tutorials::KatanaArmClient>(
    "pr2_joint_trajectory_client");

  // ── Wait for the arm to publish joint states ─────────────────────────────
  RCLCPP_INFO(arm->get_logger(), "Waiting for joint state...");
  if (!arm->waitForJointState(15.0)) {
    RCLCPP_FATAL(arm->get_logger(), "No joint state received — is the driver running?");
    rclcpp::shutdown();
    return 1;
  }

  // ── Named poses (rad) ────────────────────────────────────────────────────
  //   Post-calibration rest pose (arm folded at top after calibration)
  const std::vector<double> calibration = {-2.96, 2.14, -2.16, -1.97, -2.93};

  //   Arm pointing vertically upward
  const std::vector<double> straight_up = {0.0, 1.57, 0.0, 0.0, 0.0};

  const auto & JOINTS = katana_tutorials::KatanaArmClient::ARM_JOINT_NAMES;

  // ── Execute the sequence ─────────────────────────────────────────────────

  // Move 1: current → calibration pose
  RCLCPP_INFO(arm->get_logger(), "Move 1/3: current → calibration pose");
  {
    auto current = arm->currentJointPositions();
    auto traj    = katana_tutorials::makeTrajectory(JOINTS, current, calibration, 5.0, 6.0);
    if (!arm->sendTrajectoryAndWait(traj)) {
      RCLCPP_ERROR(arm->get_logger(), "Move 1 failed.");
      rclcpp::shutdown();
      return 1;
    }
  }
  RCLCPP_INFO(arm->get_logger(), "  Reached calibration pose.");

  // Move 2: calibration → straight up
  RCLCPP_INFO(arm->get_logger(), "Move 2/3: calibration → straight up");
  {
    auto traj = katana_tutorials::makeTrajectory(JOINTS, calibration, straight_up, 5.0, 6.0);
    if (!arm->sendTrajectoryAndWait(traj)) {
      RCLCPP_ERROR(arm->get_logger(), "Move 2 failed.");
      rclcpp::shutdown();
      return 1;
    }
  }
  RCLCPP_INFO(arm->get_logger(), "  Reached straight-up pose.");

  // Move 3: straight up → calibration
  RCLCPP_INFO(arm->get_logger(), "Move 3/3: straight up → calibration pose");
  {
    auto traj = katana_tutorials::makeTrajectory(JOINTS, straight_up, calibration, 5.0, 6.0);
    if (!arm->sendTrajectoryAndWait(traj)) {
      RCLCPP_ERROR(arm->get_logger(), "Move 3 failed.");
      rclcpp::shutdown();
      return 1;
    }
  }
  RCLCPP_INFO(arm->get_logger(), "  Sequence complete — arm back at calibration pose.");

  rclcpp::shutdown();
  return 0;
}
