// =============================================================================
// pose_runner.cpp
// =============================================================================
// Loads named arm poses from config/katana_poses.yaml, lets you pick one,
// plans to it with MoveIt (trajectory appears as a ghost in RViz), shows a
// joint-by-joint preview table, then executes on the real arm after you
// press Enter — or aborts safely if you press Ctrl+C.
//
// REQUIRES:
//   - real_hardware.launch.py  (ros2_control stack)
//   - move_group.launch.py     (MoveIt)
//   - moveit_rviz.launch.py    (optional, for the trajectory ghost preview)
//
// RUN:
//   ros2 run katana_tutorials pose_runner
// =============================================================================

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const std::vector<std::string> ARM_JOINTS = {
  "katana_motor1_pan_joint",
  "katana_motor2_lift_joint",
  "katana_motor3_lift_joint",
  "katana_motor4_lift_joint",
  "katana_motor5_wrist_roll_joint",
};

struct Pose {
  std::string name;
  std::string description;
  std::map<std::string, double> joints;
};

// ---------------------------------------------------------------------------

static std::vector<Pose> loadPoses(const std::string & path)
{
  std::vector<Pose> result;
  YAML::Node root = YAML::LoadFile(path);
  for (auto it = root["poses"].begin(); it != root["poses"].end(); ++it) {
    Pose p;
    p.name        = it->first.as<std::string>();
    p.description = it->second["description"] ?
                    it->second["description"].as<std::string>() : "";
    for (auto jt = it->second["joints"].begin();
         jt != it->second["joints"].end(); ++jt) {
      p.joints[jt->first.as<std::string>()] = jt->second.as<double>();
    }
    result.push_back(p);
  }
  return result;
}

static void printPreview(
  const std::string & pose_name,
  const std::vector<double> & current,
  const std::map<std::string, double> & target)
{
  std::cout << "\n=== Preview: " << pose_name << " ===\n";
  std::cout << std::left
            << std::setw(36) << "Joint"
            << std::setw(12) << "Current"
            << std::setw(12) << "Target"
            << "Delta\n";
  std::cout << std::string(68, '-') << "\n";
  for (std::size_t i = 0; i < ARM_JOINTS.size(); ++i) {
    double cur = (i < current.size()) ? current[i] : 0.0;
    double tgt = target.count(ARM_JOINTS[i]) ? target.at(ARM_JOINTS[i]) : cur;
    double delta = tgt - cur;
    std::cout << std::setw(36) << ARM_JOINTS[i]
              << std::fixed << std::setprecision(3)
              << std::setw(12) << cur
              << std::setw(12) << tgt
              << (delta >= 0.0 ? "+" : "") << delta << " rad\n";
  }
  std::cout << std::string(68, '-') << "\n";
}

// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "pose_runner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Spin in a background thread so MoveGroupInterface callbacks are handled.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = std::thread([&executor]() { executor.spin(); });

  auto shutdown = [&]() {
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
  };

  // ros2 launch sets stdin to /dev/null — open /dev/tty directly so the
  // interactive prompts still work regardless of how the process was started.
  std::ifstream tty_file("/dev/tty");
  std::istream & input = tty_file.is_open()
                         ? static_cast<std::istream &>(tty_file)
                         : std::cin;

  // ── Load poses ───────────────────────────────────────────────────────────
  std::string pkg = ament_index_cpp::get_package_share_directory("katana_test");
  std::string yaml_path = pkg + "/config/katana_poses.yaml";

  std::vector<Pose> poses;
  try {
    poses = loadPoses(yaml_path);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Cannot load poses file: %s", e.what());
    shutdown();
    return 1;
  }

  if (poses.empty()) {
    RCLCPP_FATAL(node->get_logger(), "No poses found in %s", yaml_path.c_str());
    shutdown();
    return 1;
  }

  // ── List and select a pose ───────────────────────────────────────────────
  std::cout << "\nAvailable poses  (edit config/katana_poses.yaml to add more):\n\n";
  for (std::size_t i = 0; i < poses.size(); ++i) {
    std::cout << "  [" << (i + 1) << "]  " << poses[i].name;
    if (!poses[i].description.empty())
      std::cout << "  —  " << poses[i].description;
    std::cout << "\n";
  }

  std::size_t choice = 0;
  while (choice < 1 || choice > poses.size()) {
    std::cout << "\nSelect pose [1-" << poses.size() << "]: " << std::flush;
    std::string line;
    if (!std::getline(input, line)) {
      shutdown();
      return 0;
    }
    try { choice = std::stoul(line); } catch (...) { choice = 0; }
  }
  const Pose & selected = poses[choice - 1];

  // Guard: robot_description_semantic must be on this node's parameter server.
  // It only arrives when started via pose_runner.launch.py, not ros2 run.
  if (!node->has_parameter("robot_description_semantic")) {
    RCLCPP_FATAL(node->get_logger(),
      "\n\n  pose_runner must be started with its launch file, not ros2 run.\n"
      "  Run:  ros2 launch katana_tutorials pose_runner.launch.py\n");
    shutdown();
    return 1;
  }

  // ── Connect to move_group ────────────────────────────────────────────────
  RCLCPP_INFO(node->get_logger(), "Connecting to move_group...");
  MoveGroupInterface move_group(node, "arm");
  move_group.setPlanningTime(10.0);

  // ── Get current joint positions ──────────────────────────────────────────
  std::vector<double> current_joints;
  auto current_state = move_group.getCurrentState(5.0);
  if (current_state) {
    current_state->copyJointGroupPositions(
      current_state->getJointModelGroup("arm"), current_joints);
  }

  // ── Print preview table ──────────────────────────────────────────────────
  printPreview(selected.name, current_joints, selected.joints);

  // ── Plan (trajectory will appear in RViz as a ghost) ────────────────────
  move_group.setJointValueTarget(selected.joints);
  std::cout << "\nPlanning...";
  std::cout.flush();
  MoveGroupInterface::Plan plan;
  auto plan_result = move_group.plan(plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    std::cout << " FAILED (code=" << plan_result.val << ").\n";
    RCLCPP_ERROR(node->get_logger(), "Planning failed — check that move_group is running "
      "and the target pose is reachable.");
    shutdown();
    return 1;
  }
  std::cout << " OK\n";
  std::cout << "Trajectory is now visible in RViz as a ghost preview.\n";

  // ── Confirm before executing ─────────────────────────────────────────────
  std::cout << "\nPress Enter to execute on the REAL ARM, Ctrl+C to abort: "
            << std::flush;
  std::string confirm;
  if (!std::getline(input, confirm)) {
    std::cout << "\nAborted.\n";
    shutdown();
    return 0;
  }

  // ── Execute ──────────────────────────────────────────────────────────────
  auto exec_result = move_group.execute(plan);
  if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
    std::cout << "Done — arm reached \"" << selected.name << "\".\n";
  } else {
    std::cout << "Execution FAILED (code=" << exec_result.val << ").\n";
  }

  shutdown();
  return (exec_result == moveit::core::MoveItErrorCode::SUCCESS) ? 0 : 1;
}
