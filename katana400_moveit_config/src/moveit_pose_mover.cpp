// =============================================================================
// moveit_pose_mover.cpp  —  MoveIt 2 interactive Cartesian commander
// =============================================================================
//
// A MoveIt 2 version of ik_pose_mover.  The full ros2_control + move_group
// stack must already be running.  Coordinates are in SI units (metres, radians)
// and MoveIt handles IK, collision checking, and smooth trajectory execution.
//
// WORKFLOW
// ────────
//   On startup: prints current TCP pose (katana_gripper_link in katana_base_link)
//
//   Menu:
//     [p]  Single pose goal   — enter X Y Z Roll Pitch Yaw (metres, rad)
//                               → plan → preview joints → confirm → execute
//     [c]  Cartesian path     — enter N waypoints one by one, then 'done'
//                               → computeCartesianPath → preview → confirm → execute
//     [j]  Print current joint positions
//     [q]  Quit
//
// UNITS
// ─────
//   X, Y, Z     — metres (e.g. 0.25 = 25 cm)
//   Roll/Pitch/Yaw — radians (e.g. 1.5708 ≈ π/2 = 90°)
//   Orientation convention: RPY applied in the order roll→pitch→yaw (extrinsic XYZ)
//
// REQUIRES
// ────────
//   ros2 launch katana400_moveit_config full_system.launch.py
//       connection_type:=tcp ip_address:=192.168.1.1 calibrate_on_startup:=true
//
// RUN
// ───
//   ros2 launch katana400_moveit_config moveit_pose_mover.launch.py
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

static constexpr const char * PLANNING_GROUP   = "arm";
static constexpr const char * END_EFFECTOR     = "katana_gripper_link";
static constexpr const char * PLANNING_FRAME   = "katana_base_link";
static constexpr double       PLANNING_TIME_S  = 10.0;
static constexpr double       EEF_STEP_M       = 0.01;   // 1 cm interpolation
static constexpr double       CART_MIN_FRACTION = 0.95;  // require ≥95% coverage

static const std::vector<std::string> ARM_JOINTS = {
  "katana_motor1_pan_joint",
  "katana_motor2_lift_joint",
  "katana_motor3_lift_joint",
  "katana_motor4_lift_joint",
  "katana_motor5_wrist_roll_joint",
};

// ─── Helpers ──────────────────────────────────────────────────────────────────

/// Convert a geometry_msgs Quaternion to Roll / Pitch / Yaw (rad).
static void quat_to_rpy(const geometry_msgs::msg::Quaternion & q,
                        double & roll, double & pitch, double & yaw)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
}

/// Convert Roll / Pitch / Yaw (rad) to a geometry_msgs Quaternion.
static geometry_msgs::msg::Quaternion rpy_to_quat(double roll, double pitch, double yaw)
{
  tf2::Quaternion tq;
  tq.setRPY(roll, pitch, yaw);
  tq.normalize();
  geometry_msgs::msg::Quaternion q;
  q.x = tq.x(); q.y = tq.y(); q.z = tq.z(); q.w = tq.w();
  return q;
}

/// Print a pose as  X=… m  Y=… m  Z=… m  R=… rad  P=… rad  Y=… rad
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

/// Print the final joint positions from a planned trajectory for preview.
static void print_joint_preview(
  const MoveGroupInterface::Plan & plan,
  const std::vector<double> & current_joints)
{
  const auto & traj = plan.trajectory.joint_trajectory;
  if (traj.points.empty()) {
    std::cout << "  (no trajectory points)\n";
    return;
  }

  const auto & last = traj.points.back();
  std::cout << "\n  Joint preview  (current → target):\n";
  std::cout << "  " << std::left << std::setw(38) << "Joint"
            << std::setw(12) << "Current"
            << std::setw(12) << "Target"
            << "Delta\n";
  std::cout << "  " << std::string(68, '-') << "\n";

  for (std::size_t i = 0; i < traj.joint_names.size(); ++i) {
    double cur = (i < current_joints.size()) ? current_joints[i] : 0.0;
    double tgt = (i < last.positions.size())  ? last.positions[i]  : 0.0;
    double delta = tgt - cur;
    std::cout << "  " << std::setw(38) << traj.joint_names[i]
              << std::fixed << std::setprecision(4)
              << std::setw(12) << cur
              << std::setw(12) << tgt
              << (delta >= 0 ? "+" : "") << delta << " rad\n";
  }
  std::cout << "  " << std::string(68, '-') << "\n";

  double dur = rclcpp::Duration(traj.points.back().time_from_start).seconds();
  std::cout << "  Trajectory duration: " << std::fixed << std::setprecision(2)
            << dur << " s  (" << traj.points.size() << " waypoints)\n";
}

/// Try to read 6 doubles from a line.  Returns false on parse error.
static bool parse6(const std::string & line,
                   double & x, double & y, double & z,
                   double & r, double & p, double & yaw)
{
  std::istringstream ss(line);
  return static_cast<bool>(ss >> x >> y >> z >> r >> p >> yaw);
}

// ─── Pose-goal mode ───────────────────────────────────────────────────────────

static void run_pose_goal(
  MoveGroupInterface & mg,
  std::istream & input,
  const rclcpp::Logger & logger)
{
  std::cout << "\n┌── Pose Goal ──────────────────────────────────────────────────┐\n"
            << "│  Enter target pose in the katana_base_link frame:             │\n"
            << "│  Format:  X   Y   Z   Roll   Pitch   Yaw  (metres, radians)  │\n"
            << "│  Example: 0.25 0.0 0.30 0.0 1.5708 0.0                       │\n"
            << "└───────────────────────────────────────────────────────────────┘\n";

  double x, y, z, r, p, yaw;
  std::string line;

  while (true) {
    std::cout << "\n  X Y Z Roll Pitch Yaw: ";
    if (!std::getline(input, line)) return;
    if (parse6(line, x, y, z, r, p, yaw)) break;
    std::cout << "  [!] Need exactly 6 numbers — try again.\n";
  }

  // Build target pose
  geometry_msgs::msg::Pose target;
  target.position.x    = x;
  target.position.y    = y;
  target.position.z    = z;
  target.orientation   = rpy_to_quat(r, p, yaw);

  std::cout << "\n  Requested:\n";
  print_pose("  Target:", target);

  // Get current joints for the preview table
  std::vector<double> cur_joints;
  auto state = mg.getCurrentState(5.0);
  if (state)
    state->copyJointGroupPositions(state->getJointModelGroup(PLANNING_GROUP), cur_joints);

  // Plan
  mg.setPoseTarget(target);
  std::cout << "\n  Planning ...";
  std::cout.flush();

  MoveGroupInterface::Plan plan;
  auto res = mg.plan(plan);

  if (res != moveit::core::MoveItErrorCode::SUCCESS) {
    std::cout << " FAILED (code=" << res.val << ").\n"
              << "  Possible reasons: target unreachable, in collision, "
                 "or outside joint limits.\n";
    RCLCPP_WARN(logger, "Planning failed with error code %d", res.val);
    mg.clearPoseTargets();
    return;
  }
  std::cout << " OK\n";

  print_joint_preview(plan, cur_joints);

  std::cout << "\n  Press Enter to execute on the REAL ARM, Ctrl+C to abort: "
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
  std::cout << "\n┌── Cartesian Path ──────────────────────────────────────────────┐\n"
            << "│  Enter waypoints one per line (X Y Z Roll Pitch Yaw, m/rad).  │\n"
            << "│  The arm will follow a STRAIGHT LINE between each pair.       │\n"
            << "│  Type  done  (or press Ctrl+D) when finished.                 │\n"
            << "└────────────────────────────────────────────────────────────────┘\n";

  // Start from current pose so the path begins exactly where the arm is.
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
    wp.position.x  = x;  wp.position.y = y;  wp.position.z = z;
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
  double fraction = mg.computeCartesianPath(
    waypoints,
    EEF_STEP_M,
    trajectory);

  std::cout << "  " << std::fixed << std::setprecision(1)
            << (fraction * 100.0) << "% of path planned.\n";

  if (fraction < CART_MIN_FRACTION) {
    std::cout << "  ✗  Path coverage is below " << static_cast<int>(CART_MIN_FRACTION * 100)
              << "% — the path may cross a singularity or reach a joint limit.\n"
              << "  Adjust the waypoints and try again.\n";
    RCLCPP_WARN(logger, "Cartesian path only %.0f%% complete", fraction * 100.0);
    return;
  }

  // Wrap as a Plan so we can print and execute it
  MoveGroupInterface::Plan cart_plan;
  cart_plan.trajectory = trajectory;

  // Print joint preview (start vs end)
  std::vector<double> cur_joints;
  auto state = mg.getCurrentState(5.0);
  if (state)
    state->copyJointGroupPositions(state->getJointModelGroup(PLANNING_GROUP), cur_joints);
  print_joint_preview(cart_plan, cur_joints);

  std::cout << "\n  Press Enter to execute on the REAL ARM, Ctrl+C to abort: "
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
    "moveit_pose_mover",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Spin in background so MoveGroupInterface callbacks are serviced.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = std::thread([&executor]() { executor.spin(); });

  auto shutdown = [&]() {
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
  };

  // Guard: robot_description_semantic must be on this node's parameter server,
  // which only happens when the node is started from the launch file.
  if (!node->has_parameter("robot_description_semantic")) {
    RCLCPP_FATAL(node->get_logger(),
      "\n\n"
      "  moveit_pose_mover must be started with its launch file, not ros2 run.\n"
      "  Run:\n"
      "    ros2 launch katana400_moveit_config moveit_pose_mover.launch.py\n"
      "  (Also make sure full_system.launch.py is running first.)\n");
    shutdown();
    return 1;
  }

  // Open /dev/tty directly so interactive prompts work even if stdin is
  // redirected (e.g. when launched via ros2 launch).
  std::ifstream tty_file("/dev/tty");
  std::istream & input = tty_file.is_open()
                         ? static_cast<std::istream &>(tty_file)
                         : std::cin;

  // ── Connect to MoveIt ───────────────────────────────────────────────────────
  std::cout << "\nConnecting to move_group (planning group: \"" << PLANNING_GROUP << "\") ...\n";
  MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setPlanningTime(PLANNING_TIME_S);
  move_group.setPoseReferenceFrame(PLANNING_FRAME);
  move_group.setEndEffectorLink(END_EFFECTOR);

  std::cout << "  Planning frame   : " << move_group.getPlanningFrame()       << "\n";
  std::cout << "  End-effector link: " << move_group.getEndEffectorLink()     << "\n";
  std::cout << "  Planner          : " << move_group.getDefaultPlannerId()    << "\n";

  // ── Banner ──────────────────────────────────────────────────────────────────
  std::cout << "\n"
            << "╔══════════════════════════════════════════════════════════════════╗\n"
            << "║      MoveIt Pose Mover  —  Katana 400 6M180                    ║\n"
            << "╠══════════════════════════════════════════════════════════════════╣\n"
            << "║  Units:  X/Y/Z in metres,  Roll/Pitch/Yaw in radians           ║\n"
            << "║  Frame:  katana_base_link (origin at arm pivot)                ║\n"
            << "║  Tip:    katana_gripper_link (centre of gripper jaw gap)       ║\n"
            << "╚══════════════════════════════════════════════════════════════════╝\n\n";

  // ── Main loop ───────────────────────────────────────────────────────────────
  std::string cmd;
  while (rclcpp::ok()) {
    // Always show current TCP pose at the prompt
    try {
      auto cur = move_group.getCurrentPose(END_EFFECTOR).pose;
      print_pose("\nCurrent TCP:", cur);
    } catch (const std::exception & e) {
      std::cout << "\n[warn] Could not read current pose: " << e.what() << "\n";
    }

    std::cout << "\n  Commands:  [p] Pose goal   [c] Cartesian path"
              << "   [j] Joint positions   [q] Quit\n"
              << "  > " << std::flush;

    if (!std::getline(input, cmd)) break;   // EOF / Ctrl+D

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
      std::cout << "\n  Current joint positions:\n";
      auto state = move_group.getCurrentState(5.0);
      if (state) {
        std::vector<double> jv;
        state->copyJointGroupPositions(state->getJointModelGroup(PLANNING_GROUP), jv);
        for (std::size_t i = 0; i < ARM_JOINTS.size() && i < jv.size(); ++i) {
          std::cout << "    " << std::left << std::setw(38) << ARM_JOINTS[i]
                    << std::fixed << std::setprecision(4) << jv[i] << " rad"
                    << "  (" << std::setprecision(1) << jv[i] * 180.0 / M_PI << "°)\n";
        }
      } else {
        std::cout << "  [!] Could not retrieve joint state.\n";
      }

    } else {
      std::cout << "  [!] Unknown command '" << cmd << "'. "
                << "Use  p / c / j / q.\n";
    }
  }

  shutdown();
  return 0;
}
