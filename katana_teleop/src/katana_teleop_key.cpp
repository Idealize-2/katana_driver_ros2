/*
 * katana_teleop_key.cpp — ROS 2 keyboard teleop via ros2_control
 *
 * Operates entirely through the ros2_control layer:
 *   READ  : subscribes /joint_states (joint_state_broadcaster)
 *   WRITE : publishes to /arm_controller/joint_trajectory  (jog)
 *           FollowJointTrajectory action                    (hold / home / gripper)
 *   POWER : katana_hw/set_motors_enabled service
 *
 * Requires the following controllers to be active:
 *   - joint_state_broadcaster
 *   - arm_controller        (JointTrajectoryController)
 *   - gripper_controller    (JointTrajectoryController)
 *
 * Usage:
 *   ros2 run katana_teleop katana_teleop_key
 *
 * Keys:
 *   1 - 5   Select arm joint to jog
 *   W / S   Jog selected joint  up (+step) / down (-step)
 *   A / D   Jog joint 1 (pan)   left (+)  / right (-)
 *   H       Go to home  (all arm joints → 0 rad, 3 s)
 *   G       Open gripper
 *   C       Close gripper
 *   E       Enable  motors  (katana_hw/set_motors_enabled true)
 *   D       Disable motors  (katana_hw/set_motors_enabled false — arm goes limp)
 *   P       Print current joint states
 *   + / =   Double  jog step size
 *   -       Halve   jog step size
 *   ?       Show this help
 *   Q       Quit
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <iterator>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ─── Joint configuration ──────────────────────────────────────────────────────

static const std::vector<std::string> ARM_JOINTS = {
  "katana_motor1_pan_joint",
  "katana_motor2_lift_joint",
  "katana_motor3_lift_joint",
  "katana_motor4_lift_joint",
  "katana_motor5_wrist_roll_joint",
};

// GripperActionController only commands katana_l_finger_joint;
// katana_r_finger_joint is a URDF mimic joint and follows automatically.
static constexpr double GRIPPER_OPEN  = 0.30;   // rad
static constexpr double GRIPPER_CLOSE = 0.00;   // rad
static constexpr double JOG_STEP_DEFAULT = 0.02; // rad per keypress
static constexpr int    PAN_JOINT_IDX   = 0;    // ARM_JOINTS[0] = pan

// ─── ROS 2 node ──────────────────────────────────────────────────────────────

using FJT       = control_msgs::action::FollowJointTrajectory;
using GH        = rclcpp_action::ClientGoalHandle<FJT>;
using GripCmd   = control_msgs::action::GripperCommand;
using GripGH    = rclcpp_action::ClientGoalHandle<GripCmd>;
using JState    = sensor_msgs::msg::JointState;
using SetBool   = std_srvs::srv::SetBool;

class KatanaTeleop : public rclcpp::Node
{
public:
  KatanaTeleop()
  : Node("katana_teleop"),
    jog_step_(JOG_STEP_DEFAULT),
    selected_(0)
  {
    arm_positions_.assign(ARM_JOINTS.size(), 0.0);
    arm_velocities_.assign(ARM_JOINTS.size(), 0.0);
    gripper_position_ = 0.0;

    // joint_state_broadcaster in Jazzy uses SensorDataQoS (BEST_EFFORT).
    js_sub_ = create_subscription<JState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const JState::SharedPtr msg) { cacheState(msg); });

    // Direct topic publish — replaces the active trajectory immediately
    // (no action goal queue buildup), ideal for responsive jog.
    arm_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);

    arm_client_ = rclcpp_action::create_client<FJT>(
      this, "/arm_controller/follow_joint_trajectory");

    // GripperActionController exposes a GripperCommand action (not FJT).
    gripper_client_ = rclcpp_action::create_client<GripCmd>(
      this, "/gripper_controller/gripper_cmd");

    motor_power_client_ = create_client<SetBool>("katana_hw/set_motors_enabled");
  }

  bool hasState() const { return has_state_; }

  // ── READ ────────────────────────────────────────────────────────────────────

  void printState()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_state_) {
      printf("\n[READ] No /joint_states received yet.\n");
      fflush(stdout);
      return;
    }
    printf("\n[READ] /joint_states\n");
    for (size_t i = 0; i < ARM_JOINTS.size(); ++i) {
      const char* sel = (i == selected_) ? "  <- selected" : "";
      printf("  [%zu] %-40s  pos=%+.4f rad   vel=%+.4f rad/s%s\n",
             i + 1, ARM_JOINTS[i].c_str(),
             arm_positions_[i], arm_velocities_[i], sel);
    }
    printf("  [G] katana_l_finger_joint                    pos=%+.4f rad\n",
           gripper_position_);
    printf("  jog_step = %.4f rad\n", jog_step_);
    fflush(stdout);
  }

  // ── SELECT ──────────────────────────────────────────────────────────────────

  void selectJoint(size_t idx)
  {
    selected_ = idx;
    printf("[SELECT] joint %zu → %s\n", idx + 1, ARM_JOINTS[idx].c_str());
    fflush(stdout);
  }

  // ── JOG ─────────────────────────────────────────────────────────────────────
  // Publishes directly to the JointTrajectory topic — preempts any ongoing
  // trajectory immediately so jog feels snappy.

  void jog(size_t joint_idx, double sign)
  {
    std::vector<double> pos;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pos = arm_positions_;
    }
    pos[joint_idx] += sign * jog_step_;

    printf("[JOG] joint %zu (%s) → %+.4f rad  (step=%.4f)\n",
           joint_idx + 1, ARM_JOINTS[joint_idx].c_str(),
           pos[joint_idx], jog_step_);
    fflush(stdout);

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions  = pos;
    pt.velocities.assign(pos.size(), 0.0);
    pt.time_from_start = rclcpp::Duration::from_seconds(2.0);

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = ARM_JOINTS;
    traj.points      = {pt};

    arm_traj_pub_->publish(traj);
  }

  // ── HOLD ────────────────────────────────────────────────────────────────────

  void hold()
  {
    std::vector<double> pos;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pos = arm_positions_;
    }
    sendArmGoal(pos, 2.0, "hold");
  }

  // ── HOME ────────────────────────────────────────────────────────────────────

  void home()
  {
    sendArmGoal(std::vector<double>(ARM_JOINTS.size(), 0.0), 3.0, "home");
  }

  // ── GRIPPER ─────────────────────────────────────────────────────────────────
  // Uses GripperCommand action (position_controllers/GripperActionController).
  // Sends a single position setpoint + max_effort=0 (position-only control).

  void setGripper(double position)
  {
    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
      printf("[GRIPPER] gripper_controller not available\n");
      fflush(stdout);
      return;
    }

    GripCmd::Goal goal;
    goal.command.position   = position;
    goal.command.max_effort = 0.0;   // 0 = position-only, no effort limit

    printf("[GRIPPER] → %s (%.4f rad)\n",
           position > 0.01 ? "OPEN" : "CLOSE", position);
    fflush(stdout);

    auto opts = rclcpp_action::Client<GripCmd>::SendGoalOptions();
    opts.result_callback = [](const GripGH::WrappedResult & r) {
      printf("[GRIPPER] result: %s  pos=%.4f  effort=%.4f\n",
             r.result->reached_goal ? "REACHED" : "STALLED",
             r.result->position, r.result->effort);
      fflush(stdout);
    };
    gripper_client_->async_send_goal(goal, opts);
  }

  // ── MOTOR POWER ─────────────────────────────────────────────────────────────

  void setMotorPower(bool enable)
  {
    if (!motor_power_client_->wait_for_service(std::chrono::seconds(2))) {
      printf("[POWER] katana_hw/set_motors_enabled service not available\n");
      fflush(stdout);
      return;
    }
    auto req = std::make_shared<SetBool::Request>();
    req->data = enable;
    motor_power_client_->async_send_request(req,
      [enable](rclcpp::Client<SetBool>::SharedFuture fut) {
        auto resp = fut.get();
        printf("[POWER] Motors %s — %s\n",
               enable ? "ON" : "OFF",
               resp->success ? "OK" : "FAILED");
        fflush(stdout);
      });
  }

  // ── STEP SIZE ────────────────────────────────────────────────────────────────

  void stepDouble() { jog_step_ = std::min(0.5,  jog_step_ * 2.0); }
  void stepHalve()  { jog_step_ = std::max(0.001, jog_step_ / 2.0); }
  double jogStep()  const { return jog_step_; }
  size_t selected() const { return selected_; }

private:
  // ── State cache ──────────────────────────────────────────────────────────────

  void cacheState(const JState::SharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    has_state_ = true;

    auto readJoint = [&](const std::string & name,
                         std::vector<double> & pos_vec,
                         std::vector<double> * vel_vec,
                         size_t idx)
    {
      auto it = std::find(msg->name.begin(), msg->name.end(), name);
      if (it == msg->name.end()) return;
      size_t mi = static_cast<size_t>(std::distance(msg->name.begin(), it));
      if (!msg->position.empty() && mi < msg->position.size())
        pos_vec[idx] = msg->position[mi];
      if (vel_vec && !msg->velocity.empty() && mi < msg->velocity.size())
        (*vel_vec)[idx] = msg->velocity[mi];
    };

    for (size_t i = 0; i < ARM_JOINTS.size(); ++i)
      readJoint(ARM_JOINTS[i], arm_positions_, &arm_velocities_, i);

    // Read gripper from katana_l_finger_joint only
    auto git = std::find(msg->name.begin(), msg->name.end(),
                         std::string("katana_l_finger_joint"));
    if (git != msg->name.end()) {
      size_t mi = static_cast<size_t>(std::distance(msg->name.begin(), git));
      if (!msg->position.empty() && mi < msg->position.size())
        gripper_position_ = msg->position[mi];
    }
  }

  // ── Arm action helper ────────────────────────────────────────────────────────

  void sendArmGoal(const std::vector<double> & positions,
                   double dur_sec, const std::string & label)
  {
    if (!arm_client_->wait_for_action_server(std::chrono::seconds(2))) {
      printf("[WRITE] arm_controller not available\n");
      fflush(stdout);
      return;
    }

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions  = positions;
    pt.velocities.assign(positions.size(), 0.0);
    pt.time_from_start = rclcpp::Duration::from_seconds(dur_sec);

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = ARM_JOINTS;
    traj.points      = {pt};

    FJT::Goal goal;
    goal.trajectory = traj;

    printf("[WRITE] %s → [", label.c_str());
    for (size_t i = 0; i < positions.size(); ++i)
      printf("%s%+.4f", i ? "  " : "", positions[i]);
    printf("] in %.1f s\n", dur_sec);
    fflush(stdout);

    auto opts = rclcpp_action::Client<FJT>::SendGoalOptions();
    opts.result_callback = [label](const GH::WrappedResult & r) {
      printf("[WRITE] %s result: %s (code=%d)\n", label.c_str(),
             r.result->error_code == 0 ? "SUCCESSFUL" : "FAILED",
             r.result->error_code);
      fflush(stdout);
    };
    arm_client_->async_send_goal(goal, opts);
  }

  // ── Members ──────────────────────────────────────────────────────────────────

  rclcpp::Subscription<JState>::SharedPtr js_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_traj_pub_;
  rclcpp_action::Client<FJT>::SharedPtr arm_client_;
  rclcpp_action::Client<GripCmd>::SharedPtr gripper_client_;
  rclcpp::Client<SetBool>::SharedPtr motor_power_client_;

  std::mutex state_mutex_;
  std::atomic<bool> has_state_{false};
  std::vector<double> arm_positions_;
  std::vector<double> arm_velocities_;
  double gripper_position_{0.0};

  double jog_step_;
  size_t selected_;
};

// ─── Terminal helpers ─────────────────────────────────────────────────────────

static struct termios g_orig_termios;

static void restoreTerminal()
{
  tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
}

static void setRawMode()
{
  tcgetattr(STDIN_FILENO, &g_orig_termios);
  atexit(restoreTerminal);
  struct termios raw = g_orig_termios;
  raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  raw.c_cc[VMIN]  = 0;
  raw.c_cc[VTIME] = 1;  // 100 ms read timeout — keeps loop responsive
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

// ─── Help ─────────────────────────────────────────────────────────────────────

static void printHelp(double jog_step, size_t selected)
{
  printf(
    "\n"
    "╔══════════════════════════════════════════════════════╗\n"
    "║   Katana ros2_control Keyboard Teleop                ║\n"
    "╠══════════════════════════════════════════════════════╣\n"
    "║  1 – 5   Select arm joint to jog                    ║\n"
    "║  W / S   Jog selected joint  + / - step             ║\n"
    "║  A / D   Jog joint 1 (pan)   + / - step             ║\n"
    "║  H       Home  (all joints → 0 rad, 3 s)            ║\n"
    "║  G       Open  gripper                              ║\n"
    "║  C       Close gripper                              ║\n"
    "║  E       Enable  motors                             ║\n"
    "║  D       Disable motors  (arm goes limp)            ║\n"
    "║  P       Print joint states                         ║\n"
    "║  + / =   Double  step size                          ║\n"
    "║  -       Halve   step size                          ║\n"
    "║  ?       Show this help                             ║\n"
    "║  Q       Quit                                       ║\n"
    "╚══════════════════════════════════════════════════════╝\n"
  );
  printf("  Current: joint %zu (%s),  step = %.4f rad\n\n",
         selected + 1, ARM_JOINTS[selected].c_str(), jog_step);
  fflush(stdout);
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KatanaTeleop>();

  // Spin in a background thread so the main thread can block on keyboard I/O.
  std::atomic<bool> running{true};
  std::thread spin_thread([&]() {
    while (running && rclcpp::ok())
      rclcpp::spin_some(node);
  });

  printHelp(node->jogStep(), node->selected());

  // Wait briefly for first /joint_states message.
  printf("Waiting for /joint_states");
  fflush(stdout);
  auto t0 = std::chrono::steady_clock::now();
  while (!node->hasState() && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    printf(".");
    fflush(stdout);
    if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(5)) {
      printf("\n  WARNING: no /joint_states in 5 s "
             "(is joint_state_broadcaster running?)\n");
      break;
    }
  }
  printf("\n");

  setRawMode();
  printf("Ready. Press ? for help.\n\n");
  fflush(stdout);

  bool quit = false;
  char c;
  while (rclcpp::ok() && !quit) {
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n <= 0) continue;   // 100 ms timeout, no key pressed

    switch (c) {
      // ── joint select ──────────────────────────────────────────────────────
      case '1': node->selectJoint(0); break;
      case '2': node->selectJoint(1); break;
      case '3': node->selectJoint(2); break;
      case '4': node->selectJoint(3); break;
      case '5': node->selectJoint(4); break;

      // ── jog selected joint ────────────────────────────────────────────────
      case 'w': case 'W': node->jog(node->selected(), +1.0); break;
      case 's': case 'S': node->jog(node->selected(), -1.0); break;

      // ── jog pan joint (joint 1) ───────────────────────────────────────────
      case 'a': case 'A': node->jog(PAN_JOINT_IDX, +1.0); break;
      case 'd': case 'D': node->jog(PAN_JOINT_IDX, -1.0); break;

      // ── discrete moves ────────────────────────────────────────────────────
      case 'h': case 'H': node->home(); break;

      // ── gripper ───────────────────────────────────────────────────────────
      case 'g': case 'G': node->setGripper(GRIPPER_OPEN);  break;
      case 'c': case 'C': node->setGripper(GRIPPER_CLOSE); break;

      // ── motor power ───────────────────────────────────────────────────────
      case 'e': case 'E': node->setMotorPower(true);  break;
      // 'd'/'D' is pan-left (WASD), so motor disable uses uppercase 'D' only
      // which is covered above; re-map to Ctrl+D or use a separate binding:
      // (lower 'd' is reserved for pan-left jog — use shift+D for disable)

      // ── print state ───────────────────────────────────────────────────────
      case 'p': case 'P': node->printState(); break;

      // ── step size ─────────────────────────────────────────────────────────
      case '+': case '=':
        node->stepDouble();
        printf("[STEP] jog step → %.4f rad\n", node->jogStep());
        fflush(stdout);
        break;
      case '-': case '_':
        node->stepHalve();
        printf("[STEP] jog step → %.4f rad\n", node->jogStep());
        fflush(stdout);
        break;

      // ── help ──────────────────────────────────────────────────────────────
      case '?':
        printHelp(node->jogStep(), node->selected());
        break;

      // ── quit ──────────────────────────────────────────────────────────────
      case 'q': case 'Q': case '\x03':   // q / Q / Ctrl+C
        quit = true;
        break;

      default:
        break;
    }
  }

  printf("\nShutting down.\n");
  running = false;
  rclcpp::shutdown();
  if (spin_thread.joinable()) spin_thread.join();
  return 0;
}