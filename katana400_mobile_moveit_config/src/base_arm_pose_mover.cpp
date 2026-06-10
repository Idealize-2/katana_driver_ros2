// =============================================================================
// base_arm_pose_mover.cpp  —  MoveIt 2 Cartesian commander for base_arm group
// =============================================================================
//
// Moves the Katana 400 end-effector to a target pose in the odom frame by
// planning over the combined "base_arm" group (5-DOF arm + mobile base).
// MoveIt solves IK jointly for both the arm joints AND the planar base
// position, so the whole system moves together to reach the gripper target.
//
// WORKFLOW
// ────────
//   On startup: prints current TCP pose (katana_gripper_link in odom)
//
//   Menu:
//     [p]  Single pose goal   — enter X Y Z Roll Pitch Yaw (metres, rad)
//                               → plan → preview joints → confirm → execute
//     [c]  Cartesian path     — enter N waypoints one by one, then 'done'
//                               → computeCartesianPath → preview → confirm → execute
//     [j]  Print current joint positions (arm + base)
//     [q]  Quit
//
// UNITS
// ─────
//   X, Y, Z        — metres,  relative to odom frame
//   Roll/Pitch/Yaw — radians  (extrinsic XYZ order)
//
// REQUIRES
// ────────
//   ros2 launch katana400_mobile_moveit_config full_system.launch.py   (or equivalent)
//
// RUN
// ───
//   ros2 launch katana400_mobile_moveit_config base_arm_pose_mover.launch.py
//
// =============================================================================

#include <atomic>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// ─── Constants ────────────────────────────────────────────────────────────────

// "base_arm" is the SRDF group that combines the planar mobile base joint and
// the 5-DOF arm chain.  MoveIt plans both simultaneously to reach a TCP target.
static constexpr const char * PLANNING_GROUP    = "base_arm";
static constexpr const char * END_EFFECTOR      = "katana_gripper_link";
static constexpr double       PLANNING_TIME_S   = 15.0;  // extra time for 8-DOF group
static constexpr double       EEF_STEP_M        = 0.01;  // 1 cm interpolation
static constexpr double       CART_MIN_FRACTION = 0.95;  // require ≥95% coverage

// ─── Helpers ──────────────────────────────────────────────────────────────────

static void quat_to_rpy(const geometry_msgs::msg::Quaternion & q,
                        double & roll, double & pitch, double & yaw)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
}

static geometry_msgs::msg::Quaternion rpy_to_quat(double roll, double pitch, double yaw)
{
  tf2::Quaternion tq;
  tq.setRPY(roll, pitch, yaw);
  tq.normalize();
  geometry_msgs::msg::Quaternion q;
  q.x = tq.x(); q.y = tq.y(); q.z = tq.z(); q.w = tq.w();
  return q;
}

static void print_pose(const char * label, const geometry_msgs::msg::Pose & p)
{
  double r, pitch, yaw;
  quat_to_rpy(p.orientation, r, pitch, yaw);
  std::cout << label
            << std::fixed << std::setprecision(4)
            << "  X=" << std::setw(8) << p.position.x << " m"
            << "  Y=" << std::setw(8) << p.position.y << " m"
            << "  Z=" << std::setw(8) << p.position.z << " m"
            << "    R=" << std::setw(8) << r     << " rad"
            << "  P=" << std::setw(8) << pitch  << " rad"
            << "  Yaw=" << std::setw(8) << yaw   << " rad"
            << "\n";
}

static void print_joint_preview(
  const MoveGroupInterface::Plan & plan,
  const std::vector<double> & current_joints,
  const std::vector<std::string> & current_names)
{
  const auto & traj = plan.trajectory.joint_trajectory;
  if (traj.points.empty()) {
    std::cout << "  (no trajectory points)\n";
    return;
  }

  const auto & last = traj.points.back();
  std::cout << "\n  Joint preview  (current → target):\n";
  std::cout << "  " << std::left << std::setw(42) << "Joint"
            << std::setw(12) << "Current"
            << std::setw(12) << "Target"
            << "Delta\n";
  std::cout << "  " << std::string(72, '-') << "\n";

  for (std::size_t i = 0; i < traj.joint_names.size(); ++i) {
    // Match trajectory joint to current-state index by name
    double cur = 0.0;
    for (std::size_t j = 0; j < current_names.size(); ++j) {
      if (current_names[j] == traj.joint_names[i]) {
        cur = current_joints[j];
        break;
      }
    }
    double tgt   = (i < last.positions.size()) ? last.positions[i] : 0.0;
    double delta = tgt - cur;
    std::cout << "  " << std::setw(42) << traj.joint_names[i]
              << std::fixed << std::setprecision(4)
              << std::setw(12) << cur
              << std::setw(12) << tgt
              << (delta >= 0 ? "+" : "") << delta << "\n";
  }
  std::cout << "  " << std::string(72, '-') << "\n";

  double dur = rclcpp::Duration(traj.points.back().time_from_start).seconds();
  std::cout << "  Trajectory duration: " << std::fixed << std::setprecision(2)
            << dur << " s  (" << traj.points.size() << " waypoints)\n";
}

static bool parse6(const std::string & line,
                   double & x, double & y, double & z,
                   double & r, double & p, double & yaw)
{
  std::istringstream ss(line);
  return static_cast<bool>(ss >> x >> y >> z >> r >> p >> yaw);
}

// ─── Get current joint state for base_arm group ───────────────────────────────

static bool get_current_joints(
  MoveGroupInterface & mg,
  std::vector<std::string> & names,
  std::vector<double> & values)
{
  auto state = mg.getCurrentState(5.0);
  if (!state) return false;
  const auto * group = state->getJointModelGroup(PLANNING_GROUP);
  if (!group) return false;
  names  = group->getVariableNames();
  values.resize(names.size());
  state->copyJointGroupPositions(group, values);
  return true;
}

// ─── Pose-goal mode ───────────────────────────────────────────────────────────

static void run_pose_goal(
  MoveGroupInterface & mg,
  std::istream & input,
  const rclcpp::Logger & logger)
{
  std::cout << "\n┌── Pose Goal ────────────────────────────────────────────────────┐\n"
            << "│  Enter target TCP pose in the planning frame:                   │\n"
            << "│  Format:  X   Y   Z   Roll   Pitch   Yaw  (metres, radians)    │\n"
            << "│  Example: 0.60 0.00 0.45 0.0 1.5708 0.0                        │\n"
            << "│  MoveIt will plan both the arm AND the mobile base together.    │\n"
            << "└─────────────────────────────────────────────────────────────────┘\n";

  double x, y, z, r, p, yaw;
  std::string line;

  while (true) {
    std::cout << "\n  X Y Z Roll Pitch Yaw: ";
    if (!std::getline(input, line)) return;
    if (parse6(line, x, y, z, r, p, yaw)) break;
    std::cout << "  [!] Need exactly 6 numbers — try again.\n";
  }

  geometry_msgs::msg::Pose target;
  target.position.x  = x;
  target.position.y  = y;
  target.position.z  = z;
  target.orientation = rpy_to_quat(r, p, yaw);

  std::cout << "\n  Requested:\n";
  print_pose("  Target:", target);

  std::vector<std::string> cur_names;
  std::vector<double>      cur_vals;
  get_current_joints(mg, cur_names, cur_vals);
  mg.setGoalOrientationTolerance(3.14);   // effectively ignore orientation
  //mg.setPoseTarget(target);
  mg.setPositionTarget(x, y, z, END_EFFECTOR);   // instead of setPoseTarget
  std::cout << "\n  Planning over base_arm group ...";
  std::cout.flush();

  MoveGroupInterface::Plan plan;
  auto res = mg.plan(plan);

  if (res != moveit::core::MoveItErrorCode::SUCCESS) {
    std::cout << " FAILED (code=" << res.val << ").\n"
              << "  Possible reasons: target unreachable, in collision, "
                 "or outside joint/workspace limits.\n";
    RCLCPP_WARN(logger, "Planning failed with error code %d", res.val);
    mg.clearPoseTargets();
    return;
  }
  std::cout << " OK\n";

  print_joint_preview(plan, cur_vals, cur_names);

  std::cout << "\n  Press Enter to execute on the REAL SYSTEM, Ctrl+C to abort: "
            << std::flush;
  std::string confirm;
  if (!std::getline(input, confirm)) {
    std::cout << "  Aborted.\n";
    mg.clearPoseTargets();
    return;
  }

  auto exec = mg.execute(plan);
  if (exec == moveit::core::MoveItErrorCode::SUCCESS) {
    std::cout << "\n  ✓  Execution complete.\n";
    auto reached = mg.getCurrentPose(END_EFFECTOR).pose;
    print_pose("  Reached:", reached);
  } else {
    std::cout << "\n  ✗  Execution FAILED (code=" << exec.val << ").\n";
    RCLCPP_ERROR(logger, "Execution failed with error code %d", exec.val);
  }
  mg.clearPoseTargets();
}

// ─── Cartesian-path mode ──────────────────────────────────────────────────────

static void run_cartesian_path(
  MoveGroupInterface & mg,
  std::istream & input,
  const rclcpp::Logger & logger)
{
  std::cout << "\n┌── Cartesian Path ───────────────────────────────────────────────┐\n"
            << "│  Enter waypoints one per line (X Y Z Roll Pitch Yaw, m/rad).   │\n"
            << "│  The TCP follows a STRAIGHT LINE between each pair.            │\n"
            << "│  The mobile base is co-optimised at each IK step.              │\n"
            << "│  Type  done  (or Ctrl+D) when finished.                        │\n"
            << "└─────────────────────────────────────────────────────────────────┘\n";

  std::vector<geometry_msgs::msg::Pose> waypoints;
  auto start_pose = mg.getCurrentPose(END_EFFECTOR).pose;
  waypoints.push_back(start_pose);
  std::cout << "\n  Start (current TCP pose):\n";
  print_pose("    WP0:", start_pose);

  std::string line;
  int wp_idx = 1;

  while (true) {
    std::cout << "\n  WP" << wp_idx << " (X Y Z Roll Pitch Yaw) or 'done': ";
    if (!std::getline(input, line)) break;
    if (line == "done" || line == "Done" || line == "DONE") break;

    double x, y, z, r, p, yaw;
    if (!parse6(line, x, y, z, r, p, yaw)) {
      std::cout << "  [!] Need 6 numbers or 'done' — try again.\n";
      continue;
    }

    geometry_msgs::msg::Pose wp;
    wp.position.x  = x; wp.position.y = y; wp.position.z = z;
    wp.orientation = rpy_to_quat(r, p, yaw);
    waypoints.push_back(wp);
    print_pose("    Added:", wp);
    ++wp_idx;
  }

  if (waypoints.size() < 2) {
    std::cout << "  No waypoints entered — returning to menu.\n";
    return;
  }

  std::cout << "\n  Computing Cartesian path ("
            << (waypoints.size() - 1) << " segment(s), EEF step = "
            << static_cast<int>(EEF_STEP_M * 1000) << " mm) ...";
  std::cout.flush();

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = mg.computeCartesianPath(waypoints, EEF_STEP_M, trajectory);

  std::cout << "  " << std::fixed << std::setprecision(1)
            << (fraction * 100.0) << "% of path planned.\n";

  if (fraction < CART_MIN_FRACTION) {
    std::cout << "  ✗  Path coverage below " << static_cast<int>(CART_MIN_FRACTION * 100)
              << "% — path may cross a singularity or joint limit.\n"
              << "  Adjust the waypoints and try again.\n";
    RCLCPP_WARN(logger, "Cartesian path only %.0f%% complete", fraction * 100.0);
    return;
  }

  MoveGroupInterface::Plan cart_plan;
  cart_plan.trajectory = trajectory;

  std::vector<std::string> cur_names;
  std::vector<double>      cur_vals;
  get_current_joints(mg, cur_names, cur_vals);
  print_joint_preview(cart_plan, cur_vals, cur_names);

  std::cout << "\n  Press Enter to execute on the REAL SYSTEM, Ctrl+C to abort: "
            << std::flush;
  std::string confirm;
  if (!std::getline(input, confirm)) {
    std::cout << "  Aborted.\n";
    return;
  }

  auto exec = mg.execute(cart_plan);
  if (exec == moveit::core::MoveItErrorCode::SUCCESS) {
    std::cout << "\n  ✓  Cartesian path complete.\n";
    auto reached = mg.getCurrentPose(END_EFFECTOR).pose;
    print_pose("  Final TCP:", reached);
  } else {
    std::cout << "\n  ✗  Execution FAILED (code=" << exec.val << ").\n";
    RCLCPP_ERROR(logger, "Cartesian path execution failed with error code %d", exec.val);
  }
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "base_arm_pose_mover",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = std::thread([&executor]() { executor.spin(); });

  auto shutdown = [&]() {
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
  };

  if (!node->has_parameter("robot_description_semantic")) {
    RCLCPP_FATAL(node->get_logger(),
      "\n\n"
      "  base_arm_pose_mover must be started with its launch file, not ros2 run.\n"
      "  Run:\n"
      "    ros2 launch katana400_mobile_moveit_config base_arm_pose_mover.launch.py\n"
      "  (Also make sure the full mobile system stack is running first.)\n");
    shutdown();
    return 1;
  }

  // Open /dev/tty directly so interactive prompts work when launched via ros2 launch.
  std::ifstream tty_file("/dev/tty");
  std::istream & input = tty_file.is_open()
                         ? static_cast<std::istream &>(tty_file)
                         : std::cin;

  std::cout << "\nConnecting to move_group (planning group: \"" << PLANNING_GROUP << "\") ...\n";
  MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setPlanningTime(PLANNING_TIME_S);
  // Do NOT call setPoseReferenceFrame("odom") — "odom" is not a URDF link;
  // it is only the virtual-joint parent frame name.  MoveIt resolves it
  // internally from the kinematic model without needing a TF lookup.
  move_group.setPoseReferenceFrame("base_footprint");   // a REAL link, not "odom"
  move_group.setWorkspace(-5.0, -5.0, -1.0, 5.0, 5.0, 2.0);
  move_group.setEndEffectorLink(END_EFFECTOR);

  const std::string planning_frame = move_group.getPlanningFrame();
  std::cout << "  Planning frame   : " << planning_frame                   << "\n";
  std::cout << "  End-effector link: " << move_group.getEndEffectorLink()  << "\n";
  std::cout << "  Planner          : " << move_group.getDefaultPlannerId() << "\n";

  std::cout << "\n"
            << "╔══════════════════════════════════════════════════════════════════╗\n"
            << "║   Base-Arm Pose Mover  —  Katana 400 6M180 on Mobile Base      ║\n"
            << "╠══════════════════════════════════════════════════════════════════╣\n"
            << "║  Group:  base_arm  (arm joints + planar mobile base)            ║\n"
            << "║  Units:  X/Y/Z in metres,  Roll/Pitch/Yaw in radians           ║\n";
  std::cout << "║  Frame:  " << std::left << std::setw(56) << planning_frame << "║\n";
  std::cout << "║  Tip:    katana_gripper_link                                    ║\n"
            << "╚══════════════════════════════════════════════════════════════════╝\n\n";

  std::string cmd;
  while (rclcpp::ok()) {
    try {
      auto cur = move_group.getCurrentPose(END_EFFECTOR).pose;
      print_pose("\nCurrent TCP:", cur);
    } catch (const std::exception & e) {
      std::cout << "\n[warn] Could not read current pose: " << e.what() << "\n";
    }

    std::cout << "\n  Commands:  [p] Pose goal   [c] Cartesian path"
              << "   [j] Joint positions   [q] Quit\n"
              << "  > " << std::flush;

    if (!std::getline(input, cmd)) break;
    if (cmd.empty()) continue;

    char c = cmd[0];

    if (c == 'q' || c == 'Q') {
      std::cout << "Exiting.\n";
      break;

    } else if (c == 'p' || c == 'P') {
      run_pose_goal(move_group, input, node->get_logger());

    } else if (c == 'c' || c == 'C') {
      run_cartesian_path(move_group, input, node->get_logger());

    } else if (c == 'j' || c == 'J') {
      std::vector<std::string> names;
      std::vector<double>      vals;
      if (get_current_joints(move_group, names, vals)) {
        std::cout << "\n  Current joint positions  (base_arm group):\n";
        for (std::size_t i = 0; i < names.size() && i < vals.size(); ++i) {
          std::cout << "    " << std::left << std::setw(42) << names[i]
                    << std::fixed << std::setprecision(4) << vals[i] << "\n";
        }
      } else {
        std::cout << "  [!] Could not retrieve joint state.\n";
      }

    } else {
      std::cout << "  [!] Unknown command '" << cmd << "'. Use  p / c / j / q.\n";
    }
  }

  shutdown();
  return 0;
}
