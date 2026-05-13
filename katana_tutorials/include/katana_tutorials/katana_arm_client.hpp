// =============================================================================
// katana_tutorials — Shared ROS 2 Action Client Header
// =============================================================================
// ROS 2 port of the original katana_tutorials package.
// Instead of ros::NodeHandle + actionlib::SimpleActionClient, this uses:
//   - rclcpp::Node
//   - rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>
//
// All IK tutorial programs include this header and extend KatanaArmClient.
// =============================================================================
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <functional>

// ROS 2 core
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// ROS 2 messages
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace katana_tutorials
{

/// Convenience alias for the ROS 2 action type
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT         = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

// ---------------------------------------------------------------------------
// KatanaArmClient — ROS 2 replacement for FollowJointTrajectoryClient
// ---------------------------------------------------------------------------
// This class wraps all boilerplate needed to:
//   1. Subscribe to /joint_states and cache the current arm position
//   2. Connect to the /katana_arm_controller/follow_joint_trajectory action server
//   3. Send a FollowJointTrajectory goal and wait for it to finish
//
// Tutorial-specific code (IK calculations etc.) lives in the .cpp files that
// inherit / use this class.
// ---------------------------------------------------------------------------

class KatanaArmClient : public rclcpp::Node
{
public:
  // Default joint names for the Katana 450 6M arm (5 arm + 2 gripper fingers)
  static const std::vector<std::string> ARM_JOINT_NAMES;

  explicit KatanaArmClient(const std::string & node_name = "katana_arm_client");

  // ── Wait / status helpers ─────────────────────────────────────────────────

  /// Block until the first /joint_states message is received (or timeout_sec).
  /// Returns true if a joint state was received, false on timeout.
  bool waitForJointState(double timeout_sec = 10.0);

  /// Send a trajectory goal and block until it is done.
  /// Returns true if the trajectory finished successfully.
  bool sendTrajectoryAndWait(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    double timeout_sec = 60.0);

  // ── Joint state access ────────────────────────────────────────────────────

  /// Returns the most recently cached joint positions (in radians).
  /// Indexed in the same order as ARM_JOINT_NAMES.
  std::vector<double> currentJointPositions() const;

  bool hasJointState() const { return got_joint_state_; }

private:
  // ROS 2 subscriptions / action client
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr        action_client_;

  // Cached state
  std::vector<double> current_positions_;
  bool                got_joint_state_{false};
  mutable std::mutex  state_mutex_;

  void jointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg);
};

// ---------------------------------------------------------------------------
// Helper: build a single-segment, 3-waypoint trajectory
//   Point 0  (t = 0 s):  current joint state  (start smoothly)
//   Point 1  (t = 5 s):  target positions
//   Point 2  (t = 10 s): target positions  (hold)
// ---------------------------------------------------------------------------
trajectory_msgs::msg::JointTrajectory makeTrajectory(
  const std::vector<std::string> & joint_names,
  const std::vector<double>      & current_positions,
  const std::vector<double>      & target_positions,
  double t1_sec = 5.0,
  double t2_sec = 10.0);

}  // namespace katana_tutorials
