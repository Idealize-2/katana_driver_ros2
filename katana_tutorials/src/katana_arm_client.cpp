// =============================================================================
// katana_tutorials — KatanaArmClient  (shared ROS 2 implementation)
// =============================================================================
#include "katana_tutorials/katana_arm_client.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace katana_tutorials
{

// ---------------------------------------------------------------------------
// Static joint name list
// ---------------------------------------------------------------------------
const std::vector<std::string> KatanaArmClient::ARM_JOINT_NAMES = {
  "katana_motor1_pan_joint",
  "katana_motor2_lift_joint",
  "katana_motor3_lift_joint",
  "katana_motor4_lift_joint",
  "katana_motor5_wrist_roll_joint",
};

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
KatanaArmClient::KatanaArmClient(const std::string & node_name)
: rclcpp::Node(node_name)
{
  // Pre-size current_positions with zeros (one per arm joint)
  current_positions_.resize(ARM_JOINT_NAMES.size(), 0.0);

  // Subscribe to /joint_states published by joint_state_broadcaster
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    std::bind(&KatanaArmClient::jointStateCB, this, std::placeholders::_1));

  // Create the action client that talks to the JointTrajectoryController
  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/arm_controller/follow_joint_trajectory");

  RCLCPP_INFO(get_logger(), "KatanaArmClient created — waiting for action server...");
  if (!action_client_->wait_for_action_server(10s)) {
    RCLCPP_WARN(get_logger(),
      "Action server /katana_arm_controller/follow_joint_trajectory not available. "
      "Is the controller_manager and arm_controller running?");
  } else {
    RCLCPP_INFO(get_logger(), "Action server ready.");
  }
}

// ---------------------------------------------------------------------------
// jointStateCB — cache the latest joint positions in radians
// ---------------------------------------------------------------------------
void KatanaArmClient::jointStateCB(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (std::size_t i = 0; i < ARM_JOINT_NAMES.size(); ++i) {
    for (std::size_t j = 0; j < msg->name.size(); ++j) {
      if (msg->name[j] == ARM_JOINT_NAMES[i] && j < msg->position.size()) {
        current_positions_[i] = msg->position[j];
      }
    }
  }
  got_joint_state_ = true;
}

// ---------------------------------------------------------------------------
// waitForJointState — spin until we receive at least one message
// ---------------------------------------------------------------------------
bool KatanaArmClient::waitForJointState(double timeout_sec)
{
  auto start = now();
  rclcpp::Rate rate(20.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(shared_from_this());
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (got_joint_state_) {
        RCLCPP_INFO(get_logger(), "Joint state received.");
        return true;
      }
    }
    double elapsed = (now() - start).seconds();
    if (elapsed >= timeout_sec) {
      RCLCPP_ERROR(get_logger(),
        "Timed out waiting for /joint_states after %.1f s", timeout_sec);
      return false;
    }
    rate.sleep();
  }
  return false;
}

// ---------------------------------------------------------------------------
// currentJointPositions — thread-safe read
// ---------------------------------------------------------------------------
std::vector<double> KatanaArmClient::currentJointPositions() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_positions_;
}

// ---------------------------------------------------------------------------
// sendTrajectoryAndWait
// ---------------------------------------------------------------------------
bool KatanaArmClient::sendTrajectoryAndWait(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  double timeout_sec)
{
  // Build the goal
  auto goal = FollowJointTrajectory::Goal();
  goal.trajectory = trajectory;
  goal.trajectory.header.stamp = now();

  RCLCPP_INFO(get_logger(), "Sending trajectory (%zu waypoints)...",
    trajectory.points.size());

  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

  // Result callback
  bool success = false;
  bool done    = false;
  send_goal_options.result_callback =
    [&](const GoalHandleFJT::WrappedResult & result) {
      done = true;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Trajectory SUCCEEDED.");
        success = true;
      } else {
        RCLCPP_ERROR(get_logger(), "Trajectory FAILED (code=%d).",
          static_cast<int>(result.code));
      }
    };

  auto goal_handle_future = action_client_->async_send_goal(goal, send_goal_options);

  // Wait for goal acceptance
  if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future,
    std::chrono::duration<double>(10.0)) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by the server.");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal handle is null — server rejected the goal.");
    return false;
  }

  // Wait for the trajectory to finish
  auto start = std::chrono::steady_clock::now();
  rclcpp::Rate rate(20.0);
  while (!done && rclcpp::ok()) {
    rclcpp::spin_some(shared_from_this());
    auto elapsed = std::chrono::steady_clock::now() - start;
    if (std::chrono::duration<double>(elapsed).count() > timeout_sec) {
      RCLCPP_ERROR(get_logger(), "Trajectory timed out after %.1f s.", timeout_sec);
      action_client_->async_cancel_goal(goal_handle);
      return false;
    }
    rate.sleep();
  }
  return success;
}

// ---------------------------------------------------------------------------
// makeTrajectory  — helper to build a 3-waypoint trajectory
// ---------------------------------------------------------------------------
trajectory_msgs::msg::JointTrajectory makeTrajectory(
  const std::vector<std::string> & joint_names,
  const std::vector<double>      & current_positions,
  const std::vector<double>      & target_positions,
  double t1_sec,
  double t2_sec)
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = joint_names;

  // Point 0: current position at t=0 (smooth start, no jerk)
  {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = current_positions;
    pt.time_from_start = rclcpp::Duration::from_seconds(0.0);
    traj.points.push_back(pt);
  }

  // Point 1: target position at t=t1_sec
  {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = target_positions;
    pt.time_from_start = rclcpp::Duration::from_seconds(t1_sec);
    traj.points.push_back(pt);
  }

  // Point 2: hold target at t=t2_sec
  {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = target_positions;
    pt.time_from_start = rclcpp::Duration::from_seconds(t2_sec);
    traj.points.push_back(pt);
  }

  return traj;
}

}  // namespace katana_tutorials
