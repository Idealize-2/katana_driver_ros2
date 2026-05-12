#include "katana/joint_trajectory_action_controller.h"
#include <fstream>
#include <iostream>
#include <cstdio>
#include <cmath>

namespace katana
{

JointTrajectoryActionController::JointTrajectoryActionController(std::shared_ptr<AbstractKatana> katana, rclcpp::Node::SharedPtr node) :
  katana_(katana), node_(node)
{
  joints_ = katana_->getJointNames();

  stopped_velocity_tolerance_ = node_->declare_parameter("stopped_velocity_tolerance", 1e-6);
  goal_constraints_.resize(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    std::string ns = std::string("constraints_") + joints_[i];
    goal_constraints_[i] = node_->declare_parameter(ns + ".goal", 0.02);
  }

  controller_state_publisher_ = node_->create_publisher<control_msgs::msg::JointTrajectoryControllerState>("katana_arm_controller/state", 1);
  
  action_server_ = rclcpp_action::create_server<FJTAction>(
    node_,
    "katana_arm_controller/follow_joint_trajectory",
    std::bind(&JointTrajectoryActionController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&JointTrajectoryActionController::handle_cancel, this, std::placeholders::_1),
    std::bind(&JointTrajectoryActionController::handle_accepted, this, std::placeholders::_1)
  );

  reset_trajectory_and_stop();
}

JointTrajectoryActionController::~JointTrajectoryActionController()
{
}

void JointTrajectoryActionController::reset_trajectory_and_stop()
{
  katana_->freezeRobot();

  rclcpp::Time time = node_->now();

  std::shared_ptr<SpecifiedTrajectory> hold_ptr = std::make_shared<SpecifiedTrajectory>(1);
  SpecifiedTrajectory &hold = *hold_ptr;
  hold[0].start_time = time.seconds() - 0.001;
  hold[0].duration = 0.0;
  hold[0].splines.resize(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
    hold[0].splines[j].coef[0] = katana_->getMotorAngles()[j];

  current_trajectory_ = hold_ptr;
}

void JointTrajectoryActionController::update()
{
  rclcpp::Time time = node_->now();

  std::vector<double> q(joints_.size()), qd(joints_.size()), qdd(joints_.size());

  std::shared_ptr<const SpecifiedTrajectory> traj_ptr = current_trajectory_;
  if (!traj_ptr) {
    RCLCPP_FATAL(node_->get_logger(), "The current trajectory can never be null");
    return;
  }

  const SpecifiedTrajectory &traj = *traj_ptr;

  if (traj.size() == 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "No segments in the trajectory");
    return;
  }

  int seg = -1;
  while (seg + 1 < (int)traj.size() && traj[seg + 1].start_time <= time.seconds())
  {
    ++seg;
  }

  if (seg == -1)
  {
    seg = 0;
  }

  for (size_t i = 0; i < q.size(); ++i)
  {
    sampleSplineWithTimeBounds(traj[seg].splines[i].coef, traj[seg].duration, time.seconds() - traj[seg].start_time,
                               q[i], qd[i], qdd[i]);
  }

  std::vector<double> error(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    error[i] = katana_->getMotorAngles()[i] - q[i];
  }

  control_msgs::msg::JointTrajectoryControllerState msg;

  for (size_t j = 0; j < joints_.size(); ++j)
    msg.joint_names.push_back(joints_[j]);
  msg.reference.positions.resize(joints_.size());
  msg.reference.velocities.resize(joints_.size());
  msg.reference.accelerations.resize(joints_.size());
  msg.feedback.positions.resize(joints_.size());
  msg.feedback.velocities.resize(joints_.size());
  msg.error.positions.resize(joints_.size());
  msg.error.velocities.resize(joints_.size());

  msg.header.stamp = time;
  for (size_t j = 0; j < joints_.size(); ++j)
  {
    msg.reference.positions[j] = q[j];
    msg.reference.velocities[j] = qd[j];
    msg.reference.accelerations[j] = qdd[j];
    msg.feedback.positions[j] = katana_->getMotorAngles()[j];
    msg.feedback.velocities[j] = katana_->getMotorVelocities()[j];
    msg.error.positions[j] = error[j];
    msg.error.velocities[j] = katana_->getMotorVelocities()[j] - qd[j];
  }

  controller_state_publisher_->publish(msg);
}

rclcpp_action::GoalResponse JointTrajectoryActionController::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FJTAction::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryActionController::handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryActionController::handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  std::thread{std::bind(&JointTrajectoryActionController::execute, this, std::placeholders::_1), goal_handle}.detach();
}

std::vector<int> JointTrajectoryActionController::makeJointsLookup(const trajectory_msgs::msg::JointTrajectory &msg)
{
  std::vector<int> lookup(joints_.size(), -1);
  for (size_t j = 0; j < joints_.size(); ++j)
  {
    for (size_t k = 0; k < msg.joint_names.size(); ++k)
    {
      if (msg.joint_names[k] == joints_[j])
      {
        lookup[j] = k;
        break;
      }
    }

    if (lookup[j] == -1)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to locate joint %s in the commanded trajectory.", joints_[j].c_str());
      return std::vector<int>();
    }
  }

  return lookup;
}

std::shared_ptr<SpecifiedTrajectory> JointTrajectoryActionController::calculateTrajectory(const trajectory_msgs::msg::JointTrajectory &msg)
{
  std::shared_ptr<SpecifiedTrajectory> new_traj_ptr;
  bool allPointsHaveVelocities = true;

  for (size_t i = 0; i < msg.points.size(); i++)
  {
    if (msg.points[i].accelerations.size() != 0 && msg.points[i].accelerations.size() != joints_.size())
    {
      RCLCPP_ERROR(node_->get_logger(), "Command point %d has %d elements for the accelerations", (int)i, (int)msg.points[i].accelerations.size());
      return new_traj_ptr;
    }
    if (msg.points[i].velocities.size() == 0)
    {
      allPointsHaveVelocities = false;
    }
    else if (msg.points[i].velocities.size() != joints_.size())
    {
      RCLCPP_ERROR(node_->get_logger(), "Command point %d has %d elements for the velocities", (int)i, (int)msg.points[i].velocities.size());
      return new_traj_ptr;
    }
    if (msg.points[i].positions.size() != joints_.size())
    {
      RCLCPP_ERROR(node_->get_logger(), "Command point %d has %d elements for the positions", (int)i, (int)msg.points[i].positions.size());
      return new_traj_ptr;
    }
  }

  std::vector<int> lookup = makeJointsLookup(msg);
  if (lookup.size() == 0) return new_traj_ptr;

  new_traj_ptr = std::make_shared<SpecifiedTrajectory>();
  SpecifiedTrajectory &new_traj = *new_traj_ptr;
  size_t steps = msg.points.size() - 1;
  if (steps <= 0) return new_traj_ptr;

  for (size_t i = 0; i < steps; i++)
  {
    Segment seg;
    seg.splines.resize(joints_.size());
    new_traj.push_back(seg);
  }

  for (size_t j = 0; j < joints_.size(); j++)
  {
    std::vector<double> times(steps + 1), positions(steps + 1), velocities(steps + 1);
    std::vector<double> durations(steps), coeff0(steps), coeff1(steps), coeff2(steps), coeff3(steps);

    for (size_t i = 0; i < steps + 1; i++)
    {
      times[i] = rclcpp::Time(msg.header.stamp).seconds() + rclcpp::Duration(msg.points[i].time_from_start).seconds();
      positions[i] = msg.points[i].positions[lookup[j]];
      if (allPointsHaveVelocities)
        velocities[i] = msg.points[i].velocities[lookup[j]];
    }

    for (size_t i = 0; i < steps; i++)
      durations[i] = times[i + 1] - times[i];

    if (allPointsHaveVelocities)
    {
      for (size_t i = 0; i < steps; ++i)
      {
        std::vector<double> coeff;
        getCubicSplineCoefficients(positions[i], velocities[i], positions[i + 1], velocities[i + 1], durations[i], coeff);
        coeff0[i] = coeff[0];
        coeff1[i] = coeff[1];
        coeff2[i] = coeff[2];
        coeff3[i] = coeff[3];
      }
    }
    else
    {
      splineCoefficients(steps, times.data(), positions.data(), coeff0.data(), coeff1.data(), coeff2.data(), coeff3.data());
    }

    for (size_t i = 0; i < steps; ++i)
    {
      new_traj[i].duration = durations[i];
      new_traj[i].start_time = times[i];
      new_traj[i].splines[j].target_position = positions[i + 1];
      new_traj[i].splines[j].coef[0] = coeff0[i];
      new_traj[i].splines[j].coef[1] = coeff1[i];
      new_traj[i].splines[j].coef[2] = coeff2[i];
      new_traj[i].splines[j].coef[3] = coeff3[i];
    }
  }
  return new_traj_ptr;
}

static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
{
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if (std::count(b.begin(), b.end(), a[i]) != 1) return false;
  for (size_t i = 0; i < b.size(); ++i)
    if (std::count(a.begin(), a.end(), b[i]) != 1) return false;
  return true;
}

bool JointTrajectoryActionController::validTrajectory(const SpecifiedTrajectory &traj)
{
  const double MAX_SPEED = 18000; // encoder per second
  const double MIN_TIME = 0.03; // seconds
  const double EPSILON = 0.000001;

  // ------- check times
  for (size_t seg = 0; seg < traj.size() - 1; seg++)
  {
    if (std::abs(traj[seg].start_time + traj[seg].duration - traj[seg + 1].start_time) > EPSILON)
    {
      RCLCPP_ERROR(node_->get_logger(), "start time and duration of segment %zu do not match next segment", seg);
      return false;
    }
  }
  for (size_t seg = 0; seg < traj.size(); seg++)
  {
    if (traj[seg].duration < MIN_TIME)
    {
      RCLCPP_ERROR(node_->get_logger(), "duration of segment %zu is too small", seg);
      return false;
    }
  }

  // ------- check conditions at t = 0 and t = N
  for (size_t j = 0; j < traj[0].splines.size(); j++)
  {
    if (std::abs(traj[0].splines[j].coef[1]) > EPSILON)
    {
      RCLCPP_ERROR(node_->get_logger(), "Velocity at t = 0 is not 0: %f (joint %zu)", traj[0].splines[j].coef[1], j);
      return false;
    }
  }

  for (size_t j = 0; j < traj[traj.size() - 1].splines.size(); j++)
  {
    size_t seg = traj.size() - 1;
    double vel_t, dummy;
    sampleSplineWithTimeBounds(traj[seg].splines[j].coef, traj[seg].duration, traj[seg].duration, dummy, vel_t, dummy);
    if (std::abs(vel_t) > EPSILON)
    {
      RCLCPP_ERROR(node_->get_logger(), "Velocity at t = N is not 0 (joint %zu)", j);
      return false;
    }
  }

  // ------- check for discontinuities between segments
  for (size_t seg = 0; seg < traj.size() - 1; seg++)
  {
    for (size_t j = 0; j < traj[seg].splines.size(); j++)
    {
      double pos_t, vel_t, acc_t;
      sampleSplineWithTimeBounds(traj[seg].splines[j].coef, traj[seg].duration, traj[seg].duration, pos_t, vel_t, acc_t);

      if (std::abs(traj[seg + 1].splines[j].coef[0] - pos_t) > EPSILON)
      {
        RCLCPP_ERROR(node_->get_logger(), "Position discontinuity at end of segment %zu (joint %zu)", seg, j);
        return false;
      }
      if (std::abs(traj[seg + 1].splines[j].coef[1] - vel_t) > EPSILON)
      {
        RCLCPP_ERROR(node_->get_logger(), "Velocity discontinuity at end of segment %zu (joint %zu)", seg, j);
        return false;
      }
      if (std::abs(2.0 * traj[seg + 1].splines[j].coef[2] - acc_t) > EPSILON)
      {
        RCLCPP_ERROR(node_->get_logger(), "Acceleration discontinuity (%f) at end of segment %zu (joint %zu)", std::abs(traj[seg + 1].splines[j].coef[2] - acc_t), seg, j);
        return false;
      }
    }
  }

  // ------- check position, speed, acceleration limits
  for (double t = traj[0].start_time; t < traj.back().start_time + traj.back().duration; t += 0.01)
  {
    // Determines which segment of the trajectory to use
    int seg = -1;
    while (seg + 1 < (int)traj.size() && traj[seg + 1].start_time <= t)
    {
      ++seg;
    }

    assert(seg >= 0);

    for (size_t j = 0; j < traj[seg].splines.size(); j++)
    {
      double pos_t, vel_t, acc_t;
      sampleSplineWithTimeBounds(traj[seg].splines[j].coef, traj[seg].duration, t - traj[seg].start_time, pos_t, vel_t,
                                 acc_t);

      // TODO later: check position limits (min/max encoders)

      if (std::abs(vel_t) > MAX_SPEED)
      {
        RCLCPP_ERROR(node_->get_logger(), "Velocity %f too high at time %f (joint %zu)", vel_t, t, j);
        return false;
      }

      // TODO later: check acceleration limits
    }
  }
  return true;
}

bool JointTrajectoryActionController::goalReached()
{
  for (size_t i = 0; i < joints_.size(); i++)
  {
    double error = current_trajectory_->back().splines[i].target_position - katana_->getMotorAngles()[i];
    if (goal_constraints_[i] > 0 && std::fabs(error) > goal_constraints_[i])
    {
      return false;
    }
  }
  return true;
}

bool JointTrajectoryActionController::allJointsStopped()
{
  for (size_t i = 0; i < joints_.size(); i++)
  {
    if (std::fabs(katana_->getMotorVelocities()[i]) > stopped_velocity_tolerance_)
      return false;
  }
  return true;
}

void JointTrajectoryActionController::execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<FJTAction::Result>();

  if (!setsEqual(joints_, goal->trajectory.joint_names))
  {
    RCLCPP_ERROR(node_->get_logger(), "Joints on incoming goal don't match our joints");
    result->error_code = control_msgs::action::FollowJointTrajectory_Result::INVALID_JOINTS;
    goal_handle->abort(result);
    return;
  }

  reset_trajectory_and_stop();

  if (goal->trajectory.points.empty())
  {
    result->error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
    goal_handle->succeed(result);
    return;
  }

  std::shared_ptr<SpecifiedTrajectory> new_traj = calculateTrajectory(goal->trajectory);
  if (!new_traj)
  {
    result->error_code = control_msgs::action::FollowJointTrajectory_Result::INVALID_GOAL;
    goal_handle->abort(result);
    return;
  }
  
  current_trajectory_ = new_traj;

  rclcpp::Rate rate(10);
  while ((rclcpp::Time(goal->trajectory.header.stamp).seconds() - node_->now().seconds()) > 0.5)
  {
    if (goal_handle->is_canceling() || !rclcpp::ok())
    {
      result->error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
      goal_handle->canceled(result);
      return;
    }
    rate.sleep();
  }

  auto isPreemptRequested = [goal_handle]() -> bool {
    return goal_handle->is_canceling();
  };

  bool success = katana_->executeTrajectory(new_traj, isPreemptRequested);
  if (!success)
  {
    result->error_code = control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED;
    goal_handle->abort(result);
    return;
  }

  rclcpp::Rate goalWait(10);
  while (rclcpp::ok())
  {
    katana_->refreshMotorStatus();

    if (katana_->someMotorCrashed())
    {
      result->error_code = control_msgs::action::FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED;
      goal_handle->abort(result);
      return;
    }

    if (katana_->allJointsReady() && allJointsStopped())
    {
      if (goalReached())
      {
        result->error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
        goal_handle->succeed(result);
        return;
      }
      else
      {
        result->error_code = control_msgs::action::FollowJointTrajectory_Result::GOAL_TOLERANCE_VIOLATED;
        goal_handle->abort(result);
        return;
      }
    }

    if (goal_handle->is_canceling())
    {
      result->error_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;
      goal_handle->canceled(result);
      return;
    }

    goalWait.sleep();
  }
}

}
