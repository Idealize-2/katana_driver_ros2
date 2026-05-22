// kni_gripper_test.cpp
// Dual-mode interactive gripper tester for Katana 400 6M180.
//
// mode:=kni (default)
//   Talks directly to the arm via KNI TCP.  Does NOT require ros2_control to be running.
//   !! Cannot run at the same time as ros2_control_node — both would fight over TCP. !!
//
// mode:=ros2_control
//   Sends FollowJointTrajectory goals to gripper_controller.
//   Requires: ros2 launch katana400_moveit_config real_hardware.launch.py ...
//
// Key map (both modes):
//   o   open  gripper  (URDF +0.30 rad / KNI openEncoders≈30500)
//   c   close gripper  (URDF  0.00 rad / KNI ≈halfway)
//   f   fully close    (URDF -0.44 rad / KNI closeEncoders≈12240)  [ros2_control mode only]
//   r   read  state    (encoder + URDF rad / /joint_states)
//   e   enable  motors [kni mode only]
//   d   disable motors [kni mode only]
//   ?   help
//   q   quit

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// KNI SDK — only needed for KNI mode but included unconditionally
#include "kniBase.h"
#include "KNI/cdlSocket.h"
#include "KNI/cplSerial.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ── types ─────────────────────────────────────────────────────────────────────

using FJT        = control_msgs::action::FollowJointTrajectory;
using GH         = rclcpp_action::ClientGoalHandle<FJT>;
using JointState = sensor_msgs::msg::JointState;

// ── constants ─────────────────────────────────────────────────────────────────

static const std::vector<std::string> GRIPPER_JOINTS = {
  "katana_l_finger_joint",
  "katana_r_finger_joint",
};

// URDF position limits (from katana_400_6m180.urdf.xacro)
static constexpr double GRIPPER_OPEN_RAD       =  0.30;   // post-calibration / open limit
static constexpr double GRIPPER_HALF_CLOSE_RAD =  0.00;   // safe partial close for testing
static constexpr double GRIPPER_FULL_CLOSE_RAD = -0.44;   // fully closed (hard limit)

// KNI motor index for the gripper (0-based: motor 6 in 1-based firmware numbering)
static constexpr int GRIPPER_MOTOR_IDX = 5;

// KNI encoder-to-URDF conversion constants (motor 6, katana6M180.cfg)
static constexpr double KNI_ANGLE_OFFSET  = -2.1503;   // angleOffset = -123.2 deg
static constexpr double KNI_DIR           =  1.0;       // rotationDirection = DIR_POSITIVE
static constexpr double KNI_ENC_PER_CYCLE =  51200.0;
static constexpr double KNI_TWO_PI        =  6.283185307179586;
static constexpr double URDF_OFFSET       =  0.7040;    // gear-corrected offset
static constexpr double URDF_FLIP         =  0.3255;    // gear ratio: 0.74 rad URDF / 2.274 rad motor

// ── terminal helpers ──────────────────────────────────────────────────────────

static struct termios g_orig_termios;

static void restore_terminal() { tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios); }

static void set_raw_mode()
{
  tcgetattr(STDIN_FILENO, &g_orig_termios);
  atexit(restore_terminal);
  struct termios raw = g_orig_termios;
  raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  raw.c_cc[VMIN]  = 0;
  raw.c_cc[VTIME] = 1;   // 100 ms read timeout
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

static void print_help(const std::string & mode)
{
  printf("\n Katana gripper tester  [mode: %s]\n", mode.c_str());
  printf(" ─────────────────────────────────────────────\n");
  printf("  o   open  gripper  (%.2f rad)\n",  GRIPPER_OPEN_RAD);
  printf("  c   close gripper  (%.2f rad, partial)\n", GRIPPER_HALF_CLOSE_RAD);
  if (mode == "kni") {
    printf("      (KNI closeGripper() uses closeEncoders from cfg)\n");
    printf("  e   enable  motors\n");
    printf("  d   disable motors — arm goes limp\n");
  } else {
    printf("  f   fully  close   (%.2f rad)\n", GRIPPER_FULL_CLOSE_RAD);
  }
  printf("  r   read  state\n");
  printf("  ?   this help\n");
  printf("  q   quit\n\n");
  fflush(stdout);
}

// ══════════════════════════════════════════════════════════════════════════════
//  KNI DIRECT MODE
// ══════════════════════════════════════════════════════════════════════════════

static int run_kni_mode(rclcpp::Node::SharedPtr node)
{
  // ── parameters ────────────────────────────────────────────────────────────
  const std::string ip   = node->get_parameter("ip_address").as_string();
  const int         port = static_cast<int>(node->get_parameter("tcp_port").as_int());
  const bool        cal  = node->get_parameter("calibrate").as_bool();

  std::string cfg = node->get_parameter("config_file").as_string();
  if (cfg.empty()) {
    try {
      cfg = ament_index_cpp::get_package_share_directory("kni")
            + "/KNI_4.3.0/configfiles400/katana6M180.cfg";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(),
        "Cannot resolve kni share dir: %s\n"
        "Pass config_file:=/absolute/path/to/katana6M180.cfg", e.what());
      return 1;
    }
  }

  RCLCPP_INFO(node->get_logger(),
    "[KNI] connecting: %s:%d   cfg: %s   calibrate: %s",
    ip.c_str(), port, cfg.c_str(), cal ? "yes" : "no");

  // ── KNI initialisation (mirrors katana_hardware_interface on_configure) ───
  std::unique_ptr<CCdlSocket>    device;
  std::unique_ptr<CCplSerialCRC> protocol;
  std::unique_ptr<CLMBase>       katana;

  try {
    device   = std::make_unique<CCdlSocket>(const_cast<char *>(ip.c_str()), port);
    protocol = std::make_unique<CCplSerialCRC>();
    protocol->init(device.get());
    katana   = std::make_unique<CLMBase>();
    katana->create(cfg.c_str(), protocol.get());
    RCLCPP_INFO(node->get_logger(), "[KNI] connected.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "[KNI] connection failed: %s", e.what());
    return 1;
  }

  if (cal) {
    try {
      RCLCPP_INFO(node->get_logger(),
        "[KNI] calibrating — arm will move to all joint limits...");
      katana->calibrate();
      katana->unBlock();
      RCLCPP_INFO(node->get_logger(), "[KNI] calibration done.");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(), "[KNI] calibration error: %s", e.what());
    }
  }

  // ── keyboard loop ─────────────────────────────────────────────────────────
  print_help("kni");
  set_raw_mode();

  bool quit = false;
  while (rclcpp::ok() && !quit) {
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) <= 0) continue;

    try {
      switch (c) {
        case 'o':
          printf("[KNI] opening gripper...\n"); fflush(stdout);
          katana->openGripper(/*waitUntilReached=*/true, /*timeout_ms=*/10000);
          printf("[KNI] open done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, /*refresh=*/true));
          fflush(stdout);
          break;

        case 'c':
          printf("[KNI] closing gripper (to closeEncoders from cfg)...\n");
          fflush(stdout);
          katana->closeGripper(/*waitUntilReached=*/true, /*timeout_ms=*/10000);
          printf("[KNI] close done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, /*refresh=*/true));
          fflush(stdout);
          break;

        case 'r': {
          int enc = katana->getMotorEncoders(GRIPPER_MOTOR_IDX, /*refresh=*/true);
          double kni_rad = KNI_ANGLE_OFFSET + KNI_DIR * (enc / KNI_ENC_PER_CYCLE) * KNI_TWO_PI;
          double urdf    = (kni_rad - URDF_OFFSET) * URDF_FLIP;
          printf("[KNI] gripper  encoder=%-6d  kni_rad=%+.4f  urdf=%+.4f rad\n",
                 enc, kni_rad, urdf);
          fflush(stdout);
          break;
        }

        case 'e':
          katana->switchRobotOn();
          printf("[KNI] motors enabled.\n"); fflush(stdout);
          break;

        case 'd':
          katana->freezeRobot();
          printf("[KNI] motors frozen — arm limp.\n"); fflush(stdout);
          break;

        case '?':
          print_help("kni");
          break;

        case 'q': case '\x03':
          quit = true;
          break;

        default:
          break;
      }
    } catch (const std::exception & e) {
      printf("[KNI] ERROR: %s\n", e.what()); fflush(stdout);
    }
  }

  // ── cleanup ───────────────────────────────────────────────────────────────
  RCLCPP_INFO(node->get_logger(), "[KNI] shutting down...");
  try { katana->freezeRobot();    } catch (...) {}
  try { katana->switchRobotOff(); } catch (...) {}
  katana.reset();
  protocol.reset();
  device.reset();
  return 0;
}

// ══════════════════════════════════════════════════════════════════════════════
//  ros2_control MODE
// ══════════════════════════════════════════════════════════════════════════════

class GripperControlNode : public rclcpp::Node
{
public:
  GripperControlNode()
  : Node("katana_kni_gripper_test")
  {
    finger_pos_.assign(GRIPPER_JOINTS.size(), 0.0);

    // SensorDataQoS — matches joint_state_broadcaster's BEST_EFFORT publisher
    js_sub_ = create_subscription<JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mx_);
        for (size_t j = 0; j < GRIPPER_JOINTS.size(); ++j) {
          for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == GRIPPER_JOINTS[j]) {
              finger_pos_[j] = msg->position[i];
              break;
            }
          }
        }
        has_state_ = true;
      });

    client_ = rclcpp_action::create_client<FJT>(
      this, "/gripper_controller/follow_joint_trajectory");
  }

  void send_gripper(double rad, const std::string & label)
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
      printf("[R2C] gripper_controller not available. Is the hardware stack running?\n");
      fflush(stdout);
      return;
    }

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions  = {rad, rad};
    pt.velocities = {0.0, 0.0};
    pt.time_from_start = rclcpp::Duration::from_seconds(1.5);

    FJT::Goal goal;
    goal.trajectory.joint_names = GRIPPER_JOINTS;
    goal.trajectory.points      = {pt};

    printf("[R2C] %s → %.4f rad ...\n", label.c_str(), rad); fflush(stdout);

    auto opts = rclcpp_action::Client<FJT>::SendGoalOptions();
    opts.result_callback = [label](const GH::WrappedResult & r) {
      printf("[R2C] %s: %s  (error_code=%d)\n",
             label.c_str(),
             r.result->error_code == 0 ? "OK" : "FAILED",
             r.result->error_code);
      fflush(stdout);
    };
    client_->async_send_goal(goal, opts);
  }

  void print_state()
  {
    std::lock_guard<std::mutex> lk(mx_);
    if (!has_state_) {
      printf("[R2C] no /joint_states received yet.\n"); fflush(stdout);
      return;
    }
    printf("[R2C] finger joint positions:\n");
    for (size_t i = 0; i < GRIPPER_JOINTS.size(); ++i) {
      printf("  %-32s  pos=%+.4f rad\n",
             GRIPPER_JOINTS[i].c_str(), finger_pos_[i]);
    }
    fflush(stdout);
  }

private:
  rclcpp::Subscription<JointState>::SharedPtr   js_sub_;
  rclcpp_action::Client<FJT>::SharedPtr         client_;
  std::vector<double>                           finger_pos_;
  std::mutex                                    mx_;
  bool                                          has_state_{false};
};

static int run_ros2_control_mode()
{
  auto node  = std::make_shared<GripperControlNode>();
  auto exec  = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node);
  std::thread spin_thread([&exec]() { exec->spin(); });

  print_help("ros2_control");
  set_raw_mode();

  bool quit = false;
  while (rclcpp::ok() && !quit) {
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) <= 0) continue;

    switch (c) {
      case 'o': node->send_gripper(GRIPPER_OPEN_RAD,       "open");        break;
      case 'c': node->send_gripper(GRIPPER_HALF_CLOSE_RAD, "close");       break;
      case 'f': node->send_gripper(GRIPPER_FULL_CLOSE_RAD, "full-close");  break;
      case 'r': node->print_state();                                        break;
      case '?': print_help("ros2_control");                                 break;
      case 'q': case '\x03': quit = true;                                   break;
      default: break;
    }
  }

  exec->cancel();
  spin_thread.join();
  return 0;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Read mode from a temporary node so it appears in `ros2 param list`
  auto tmp = rclcpp::Node::make_shared("katana_kni_gripper_test");
  tmp->declare_parameter("mode",        std::string("kni"));
  tmp->declare_parameter("ip_address",  std::string("192.168.1.1"));
  tmp->declare_parameter("tcp_port",    5566);
  tmp->declare_parameter("config_file", std::string(""));
  tmp->declare_parameter("calibrate",   false);

  const std::string mode = tmp->get_parameter("mode").as_string();

  int ret = 0;
  if (mode == "kni") {
    ret = run_kni_mode(tmp);
  } else if (mode == "ros2_control") {
    tmp.reset();
    rclcpp::shutdown();
    rclcpp::init(argc, argv);
    ret = run_ros2_control_mode();
  } else {
    RCLCPP_ERROR(tmp->get_logger(),
      "Unknown mode '%s'. Valid: 'kni' (default) or 'ros2_control'.", mode.c_str());
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}
