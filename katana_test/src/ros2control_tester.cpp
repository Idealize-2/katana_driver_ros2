// ros2control_tester.cpp
// Interactive keyboard tester for ros2_control read/write on the Katana arm.
//
// READ  path: subscribes /joint_states (fed by joint_state_broadcaster)
// WRITE path: sends FollowJointTrajectory goals to arm_controller / gripper_controller
//
// Key map:
//   r        READ  – print all joint positions & velocities
//   h        WRITE – hold current position (2 s trajectory)
//   0        WRITE – move all arm joints to 0 rad (home, 3 s)
//   1-5      Select arm joint for jogging
//   + / =    Jog selected joint +0.01 rad
//   -        Jog selected joint -0.01 rad
//   g        WRITE – open gripper
//   c        WRITE – close gripper
//   d        POWER – disable motors (arm goes limp, can be moved manually)
//   e        POWER – enable  motors (arm holds current position)
//   ?        Show key map
//   q        Quit

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
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

// ── constants ─────────────────────────────────────────────────────────────────

using FJT        = control_msgs::action::FollowJointTrajectory;
using GH         = rclcpp_action::ClientGoalHandle<FJT>;
using JointState = sensor_msgs::msg::JointState;
using SetBool    = std_srvs::srv::SetBool;

static const std::vector<std::string> ARM_JOINTS = {
  "katana_motor1_pan_joint",
  "katana_motor2_lift_joint",
  "katana_motor3_lift_joint",
  "katana_motor4_lift_joint",
  "katana_motor5_wrist_roll_joint",
};

static const std::vector<std::string> GRIPPER_JOINTS = {
  "katana_l_finger_joint",
  "katana_r_finger_joint",
};

static constexpr double GRIPPER_OPEN  = 0.30;  // rad
static constexpr double GRIPPER_CLOSE = 0.00;  // rad
static constexpr double JOG_STEP      = 0.01;  // rad per keypress

// ── node ──────────────────────────────────────────────────────────────────────

class KatanaTester : public rclcpp::Node
{
public:
  KatanaTester() : Node("katana_ros2control_tester")
  {
    arm_positions_.assign(ARM_JOINTS.size(), 0.0);
    arm_velocities_.assign(ARM_JOINTS.size(), 0.0);
    gripper_positions_.assign(GRIPPER_JOINTS.size(), 0.0);

    // joint_state_broadcaster in Jazzy publishes with SensorDataQoS (BEST_EFFORT).
    // A RELIABLE subscriber won't match — use the same QoS as the publisher.
    js_sub_ = create_subscription<JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const JointState::SharedPtr msg) { cache_state(msg); });

    // Topic publisher for jog — publishes JointTrajectory directly which
    // REPLACES the current trajectory immediately (no action queue buildup).
    arm_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);

    arm_client_ = rclcpp_action::create_client<FJT>(
      this, "/arm_controller/follow_joint_trajectory");

    gripper_client_ = rclcpp_action::create_client<FJT>(
      this, "/gripper_controller/follow_joint_trajectory");

    motor_power_client_ = create_client<SetBool>("katana_hw/set_motors_enabled");
  }

  bool has_state() const { return has_state_; }

  // ── READ ──────────────────────────────────────────────────────────────────

  void print_state()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_state_) {
      printf("\n[READ] No /joint_states received yet.\n");
      fflush(stdout);
      return;
    }
    printf("\n[READ] /joint_states\n");
    for (size_t i = 0; i < ARM_JOINTS.size(); ++i) {
      const char* sel = (i == selected_) ? " ← selected" : "";
      printf("  [%zu] %-40s  pos=%+.4f rad   vel=%+.4f rad/s%s\n",
             i + 1, ARM_JOINTS[i].c_str(),
             arm_positions_[i], arm_velocities_[i], sel);
    }
    for (size_t i = 0; i < GRIPPER_JOINTS.size(); ++i) {
      printf("  [G] %-40s  pos=%+.4f rad\n",
             GRIPPER_JOINTS[i].c_str(), gripper_positions_[i]);
    }
    fflush(stdout);
  }

  // ── SELECT ────────────────────────────────────────────────────────────────

  void select_joint(size_t idx)
  {
    selected_ = idx;
    printf("[SELECT] joint %zu → %s\n", idx + 1, ARM_JOINTS[idx].c_str());
    fflush(stdout);
  }

  // ── WRITE helpers ─────────────────────────────────────────────────────────

  void hold()
  {
    std::vector<double> pos;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pos = arm_positions_;
    }
    send_arm(pos, 2.0, "hold");
  }

  void home()
  {
    send_arm(std::vector<double>(ARM_JOINTS.size(), 0.0), 3.0, "home");
  }

  void jog(double delta)
  {
    std::vector<double> pos;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pos = arm_positions_;
    }
    pos[selected_] += delta;
    printf("[JOG] joint %zu (%s) → %+.4f rad\n",
           selected_ + 1, ARM_JOINTS[selected_].c_str(), pos[selected_]);
    fflush(stdout);

    // Publish directly to the topic — this REPLACES the active trajectory
    // immediately instead of queuing behind previous goals (action interface).
    // Use a long duration so the arm has time to reach the target even if
    // the user presses quickly.
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions  = pos;
    pt.velocities.assign(pos.size(), 0.0);
    pt.time_from_start = rclcpp::Duration::from_seconds(3.0);

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = ARM_JOINTS;
    traj.points      = {pt};

    arm_traj_pub_->publish(traj);
  }

  void send_arm(const std::vector<double>& positions,
                double dur_sec, const std::string& label)
  {
    if (!arm_client_->wait_for_action_server(std::chrono::seconds(2))) {
      printf("[WRITE] arm_controller not available\n");
      fflush(stdout);
      return;
    }

    auto goal = make_goal(ARM_JOINTS, positions, dur_sec);

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

  void send_gripper(double open_pos)
  {
    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
      printf("[WRITE] gripper_controller not available\n");
      fflush(stdout);
      return;
    }

    auto goal = make_goal(GRIPPER_JOINTS, {open_pos, open_pos}, 1.5);

    printf("[WRITE] gripper → %s (%.4f rad)\n",
           open_pos > 0.01 ? "OPEN" : "CLOSE", open_pos);
    fflush(stdout);

    auto opts = rclcpp_action::Client<FJT>::SendGoalOptions();
    opts.result_callback = [](const GH::WrappedResult & r) {
      printf("[WRITE] gripper result: %s\n",
             r.result->error_code == 0 ? "SUCCESSFUL" : "FAILED");
      fflush(stdout);
    };
    gripper_client_->async_send_goal(goal, opts);
  }

  void set_motors_enabled(bool enable)
  {
    if (!motor_power_client_->wait_for_service(std::chrono::seconds(2))) {
      printf("[POWER] katana_hw/set_motors_enabled service not available\n");
      fflush(stdout);
      return;
    }
    auto req = std::make_shared<SetBool::Request>();
    req->data = enable;
    motor_power_client_->async_send_request(req,
      [enable](rclcpp::Client<SetBool>::SharedFuture future) {
        auto resp = future.get();
        printf("[POWER] Motors %s — %s\n",
               enable ? "ON" : "OFF",
               resp->success ? "OK" : "FAILED");
        fflush(stdout);
      });
  }

private:
  // ── internal ──────────────────────────────────────────────────────────────

  void cache_state(const JointState::SharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    has_state_ = true;

    auto read_joint = [&](const std::string & name,
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
      read_joint(ARM_JOINTS[i], arm_positions_, &arm_velocities_, i);
    for (size_t i = 0; i < GRIPPER_JOINTS.size(); ++i)
      read_joint(GRIPPER_JOINTS[i], gripper_positions_, nullptr, i);
  }

  static FJT::Goal make_goal(const std::vector<std::string> & joints,
                              const std::vector<double> & positions,
                              double dur_sec)
  {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions  = positions;
    pt.velocities.assign(positions.size(), 0.0);
    pt.time_from_start = rclcpp::Duration::from_seconds(dur_sec);

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joints;
    traj.points      = {pt};

    FJT::Goal goal;
    goal.trajectory = traj;
    return goal;
  }

  rclcpp::Subscription<JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_traj_pub_;
  rclcpp_action::Client<FJT>::SharedPtr arm_client_;
  rclcpp_action::Client<FJT>::SharedPtr gripper_client_;
  rclcpp::Client<SetBool>::SharedPtr motor_power_client_;

  std::mutex state_mutex_;
  std::atomic<bool> has_state_{false};
  std::vector<double> arm_positions_;
  std::vector<double> arm_velocities_;
  std::vector<double> gripper_positions_;
  size_t selected_{0};
};

// ── terminal raw mode ─────────────────────────────────────────────────────────

static struct termios g_orig_termios;

static void restore_terminal()
{
  tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
}

static void set_raw_mode()
{
  tcgetattr(STDIN_FILENO, &g_orig_termios);
  atexit(restore_terminal);
  struct termios raw = g_orig_termios;
  raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  raw.c_cc[VMIN]  = 0;
  raw.c_cc[VTIME] = 1;   // 100 ms read timeout – keeps loop responsive
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

// ── help ─────────────────────────────────────────────────────────────────────

static void print_help()
{
  printf(
    "\n"
    "╔══════════════════════════════════════════════════╗\n"
    "║   Katana ros2_control interactive tester         ║\n"
    "╠══════════════════════════════════════════════════╣\n"
    "║  r        READ  /joint_states                    ║\n"
    "║  h        WRITE hold current position (2 s)      ║\n"
    "║  0        WRITE move all arm joints to 0 (3 s)   ║\n"
    "║  1 – 5   Select arm joint to jog                 ║\n"
    "║  + / =    Jog selected joint  +0.01 rad          ║\n"
    "║  -        Jog selected joint  -0.01 rad          ║\n"
    "║  g        WRITE open gripper                     ║\n"
    "║  c        WRITE close gripper                    ║\n"
    "║  d        POWER disable motors (arm goes limp)   ║\n"
    "║  e        POWER enable  motors (hold position)   ║\n"
    "║  ?        Show this help                         ║\n"
    "║  q        Quit                                   ║\n"
    "╚══════════════════════════════════════════════════╝\n\n"
  );
  fflush(stdout);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KatanaTester>();

  // Spin the node in a background thread so the main thread can block on keyboard.
  std::atomic<bool> running{true};
  std::thread spin_thread([&]() {
    while (running && rclcpp::ok())
      rclcpp::spin_some(node);
  });

  print_help();

  // Wait for first joint state message.
  printf("Waiting for /joint_states");
  fflush(stdout);
  auto t0 = std::chrono::steady_clock::now();
  while (!node->has_state() && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    printf(".");
    fflush(stdout);
    if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(5)) {
      printf("\n  WARNING: no /joint_states in 5 s "
             "(is joint_state_broadcaster active?)\n");
      break;
    }
  }
  printf("\n");

  set_raw_mode();

  printf("Ready. Press ? for help.\n\n");
  fflush(stdout);

  bool quit = false;
  char c;
  while (rclcpp::ok() && !quit) {
    ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n <= 0) continue;  // timeout, no key pressed

    switch (c) {
      case 'r':              node->print_state();            break;
      case 'h':              node->hold();                   break;
      case '0':              node->home();                   break;
      case '1':              node->select_joint(0);          break;
      case '2':              node->select_joint(1);          break;
      case '3':              node->select_joint(2);          break;
      case '4':              node->select_joint(3);          break;
      case '5':              node->select_joint(4);          break;
      case '+': case '=':    node->jog(+JOG_STEP);           break;
      case '-':              node->jog(-JOG_STEP);           break;
      case 'g':              node->send_gripper(GRIPPER_OPEN);      break;
      case 'c':              node->send_gripper(GRIPPER_CLOSE);     break;
      case 'd':              node->set_motors_enabled(false);        break;
      case 'e':              node->set_motors_enabled(true);         break;
      case '?':              print_help();                           break;
      case 'q': case '\x03': quit = true;                    break;  // q or Ctrl+C
      default:                                               break;
    }
  }

  printf("\nShutting down.\n");
  running = false;
  rclcpp::shutdown();
  if (spin_thread.joinable()) spin_thread.join();
  return 0;
}
