#ifndef JOINT_TRAJECTORY_ACTION_CONTROLLER_H__
#define JOINT_TRAJECTORY_ACTION_CONTROLLER_H__

#include <vector>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <katana/AbstractKatana.h>
#include <katana/SpecifiedTrajectory.h>
#include <katana/spline_functions.h>

namespace katana
{

class JointTrajectoryActionController
{
public:
  using FJTAction = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FJTAction>;

  JointTrajectoryActionController(std::shared_ptr<AbstractKatana> katana, rclcpp::Node::SharedPtr node);
  virtual ~JointTrajectoryActionController();

  void reset_trajectory_and_stop();
  void update();

private:
  std::vector<std::string> joints_;
  std::shared_ptr<AbstractKatana> katana_;
  rclcpp::Node::SharedPtr node_;

  double stopped_velocity_tolerance_;
  std::vector<double> goal_constraints_;

  rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr controller_state_publisher_;
  rclcpp_action::Server<FJTAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FJTAction::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);

  void execute(const std::shared_ptr<GoalHandleFJT> goal_handle);

  std::shared_ptr<SpecifiedTrajectory> current_trajectory_;

  std::shared_ptr<SpecifiedTrajectory> calculateTrajectory(const trajectory_msgs::msg::JointTrajectory &msg);

  std::vector<int> makeJointsLookup(const trajectory_msgs::msg::JointTrajectory &msg);
  bool validTrajectory(const SpecifiedTrajectory &traj);
  bool goalReached();
  bool allJointsStopped();
};
}
#endif
