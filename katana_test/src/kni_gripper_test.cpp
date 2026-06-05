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
// KNI mode key map:
//   HIGH-LEVEL GRIPPER API
//     o   openGripper()
//     c   closeGripper()
//     g   getGripperParameters          (read open/close enc + isPresent)
//     G   setGripperParameters(o, c)    (override open/close enc limits)
//
//   MOVE MOTOR 6 — by encoder (motor idx 5, 0-based)
//     1   moveMotorToEnc(enc)           absolute encoder, wait
//     2   moveMotorByEnc(delta)         relative +/- encoder, wait
//     5   inc(dif)                      +dif encoder, wait (old-style)
//     6   dec(dif)                      -dif encoder, wait (old-style)
//     7   mov(enc)                      absolute encoder, wait (old-style)
//
//   MOVE MOTOR 6 — by angle
//     3   moveMotorTo(rad)              absolute KNI radians, wait
//     4   moveMotorBy(delta_rad)        relative KNI radians, wait
//     8   movDegrees(deg)               absolute KNI degrees, wait (old-style)
//     9   incDegrees(dif_deg)           +dif KNI degrees, wait (old-style)
//     0   decDegrees(dif_deg)           -dif KNI degrees, wait (old-style)
//
//   SPLINE (low-level — Hermite cubic, zero velocity at both ends)
//     s   sendSplineToMotor(5, ...) + startSplineMovement(true, 1)
//
//   PER-MOTOR POWER (motor 6 only)
//     E   switchMotorOn(5)
//     X   switchMotorOff(5)
//     F   freezeMotor(5)
//
//   ROBOT-WIDE POWER
//     e   switchRobotOn()
//     d   freezeRobot()   — arm goes limp
//
//   INFO / LIMITS (motor 6)
//     r   read encoder + URDF rad conversion
//     v   velocity, accel, controller type, force (all at once)
//     V   setMotorVelocityLimit(5, vel)
//     A   setMotorAccelerationLimit(5, acc)
//     w   waitForMotor(5, enc, tol=100, timeout=10 s)
//
//   ros2_control mode key map (unchanged):
//     o   open  (0.30 rad)
//     c   close (0.00 rad, partial)
//     f   fully close (-0.44 rad)
//     r   read /joint_states
//
//   Both modes:
//     ?   help
//     q   quit

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
#include <cmath>
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
static constexpr double GRIPPER_OPEN_RAD       =  0.30;
static constexpr double GRIPPER_HALF_CLOSE_RAD =  0.00;
static constexpr double GRIPPER_FULL_CLOSE_RAD = -0.44;

// KNI motor index for the gripper (0-based: motor 6 in 1-based firmware numbering)
static constexpr int GRIPPER_MOTOR_IDX = 5;

// KNI encoder-to-URDF conversion constants (motor 6, katana6M180.cfg)
static constexpr double KNI_ANGLE_OFFSET  = -2.1503;
static constexpr double KNI_DIR           =  1.0;
static constexpr double KNI_ENC_PER_CYCLE =  51200.0;
static constexpr double KNI_TWO_PI        =  6.283185307179586;
static constexpr double URDF_OFFSET       =  0.7040;
static constexpr double URDF_FLIP         =  0.3255;

// Spline segment duration used by the production driver (matches kSplineT = 50 = 500 ms)
static constexpr int SPLINE_T = 50;

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

// Temporarily exit raw mode to read a typed integer from the user.
static bool prompt_int(const char * label, int & out)
{
  restore_terminal();
  printf("  enter %s: ", label); fflush(stdout);
  int r = scanf("%d", &out);
  int ch; while ((ch = getchar()) != '\n' && ch != EOF) {}
  set_raw_mode();
  return r == 1;
}

// Temporarily exit raw mode to read a typed double from the user.
static bool prompt_double(const char * label, double & out)
{
  restore_terminal();
  printf("  enter %s: ", label); fflush(stdout);
  int r = scanf("%lf", &out);
  int ch; while ((ch = getchar()) != '\n' && ch != EOF) {}
  set_raw_mode();
  return r == 1;
}

static void print_help(const std::string & mode)
{
  printf("\n Katana gripper tester  [mode: %s]  — motor 6 (idx %d)\n",
         mode.c_str(), GRIPPER_MOTOR_IDX);
  printf(" ─────────────────────────────────────────────────────────\n");

  if (mode == "kni") {
    printf(" HIGH-LEVEL GRIPPER API\n");
    printf("   o   openGripper()                    → open to openEncoders (cfg)\n");
    printf("   c   closeGripper()                   → close to closeEncoders (cfg)\n");
    printf("   g   getGripperParameters             → print open/close enc + isPresent\n");
    printf("   G   setGripperParameters(open,close) → override open/close enc limits\n");
    printf("\n MOVE MOTOR 6 — by encoder\n");
    printf("   1   moveMotorToEnc(enc)              absolute encoder, wait\n");
    printf("   2   moveMotorByEnc(delta)            relative +/- encoder, wait\n");
    printf("   5   inc(dif)                         +dif encoder, wait (old-style)\n");
    printf("   6   dec(dif)                         -dif encoder, wait (old-style)\n");
    printf("   7   mov(enc)                         absolute encoder, wait (old-style)\n");
    printf("\n MOVE MOTOR 6 — by angle (KNI units, NOT URDF)\n");
    printf("   3   moveMotorTo(rad)                 absolute KNI radians, wait\n");
    printf("   4   moveMotorBy(delta_rad)           relative KNI radians, wait\n");
    printf("   8   movDegrees(deg)                  absolute KNI degrees, wait\n");
    printf("   9   incDegrees(dif_deg)              +dif KNI degrees, wait\n");
    printf("   0   decDegrees(dif_deg)              -dif KNI degrees, wait\n");
    printf("\n SPLINE — Hermite cubic, zero vel at both ends, T=%d (~500 ms)\n", SPLINE_T);
    printf("   s   sendSplineToMotor(5,...) + startSplineMovement(true,1)\n");
    printf("\n PER-MOTOR POWER (motor 6 only)\n");
    printf("   E   switchMotorOn(5)\n");
    printf("   X   switchMotorOff(5)\n");
    printf("   F   freezeMotor(5)\n");
    printf("\n ROBOT-WIDE POWER\n");
    printf("   e   switchRobotOn()  — enable all motors\n");
    printf("   d   freezeRobot()   — freeze all (arm goes limp)\n");
    printf("\n INFO / LIMITS (motor 6)\n");
    printf("   r   read encoder + URDF rad conversion\n");
    printf("   v   velocity, accel, controllerType, force (all at once)\n");
    printf("   V   setMotorVelocityLimit(5, vel)\n");
    printf("   A   setMotorAccelerationLimit(5, acc)\n");
    printf("   w   waitForMotor(5, enc, tol=100, timeout=10 s)\n");
  } else {
    printf("   o   open  gripper  (%.2f rad)\n", GRIPPER_OPEN_RAD);
    printf("   c   close gripper  (%.2f rad, partial)\n", GRIPPER_HALF_CLOSE_RAD);
    printf("   f   fully  close   (%.2f rad)\n", GRIPPER_FULL_CLOSE_RAD);
    printf("   r   read /joint_states\n");
  }

  printf("\n   ?   this help\n");
  printf("   q   quit\n\n");
  fflush(stdout);
}

// ── encoder / rad conversion helper ──────────────────────────────────────────

static double enc_to_urdf(int enc)
{
  double kni_rad = KNI_ANGLE_OFFSET + KNI_DIR * (enc / KNI_ENC_PER_CYCLE) * KNI_TWO_PI;
  return (kni_rad - URDF_OFFSET) * URDF_FLIP;
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

  // ── KNI initialisation ────────────────────────────────────────────────────
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

        // ── High-level gripper API ─────────────────────────────────────────

        case 'o':
          printf("[KNI] openGripper(wait=true, timeout=10 s)...\n"); fflush(stdout);
          katana->openGripper(/*waitUntilReached=*/true, /*timeout_ms=*/10000);
          printf("[KNI] open done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, /*refresh=*/true));
          fflush(stdout);
          break;

        case 'c':
          printf("[KNI] closeGripper(wait=true, timeout=10 s)...\n"); fflush(stdout);
          katana->closeGripper(/*waitUntilReached=*/true, /*timeout_ms=*/10000);
          printf("[KNI] close done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, /*refresh=*/true));
          fflush(stdout);
          break;

        case 'g': {
          bool isPresent; int openEnc, closeEnc;
          katana->getGripperParameters(isPresent, openEnc, closeEnc);
          printf("[KNI] getGripperParameters: isPresent=%s  openEnc=%-6d  closeEnc=%d\n",
                 isPresent ? "true" : "false", openEnc, closeEnc);
          fflush(stdout);
          break;
        }

        case 'G': {
          int openEnc, closeEnc;
          if (!prompt_int("openEncoders", openEnc))   break;
          if (!prompt_int("closeEncoders", closeEnc)) break;
          katana->setGripperParameters(true, openEnc, closeEnc);
          printf("[KNI] setGripperParameters(true, %d, %d) done.\n", openEnc, closeEnc);
          fflush(stdout);
          break;
        }

        // ── Move motor 6 — by encoder ──────────────────────────────────────

        case '1': {
          int enc;
          if (!prompt_int("target encoder (abs)", enc)) break;
          printf("[KNI] moveMotorToEnc(5, %d, wait=true, tol=100, 10 s)...\n", enc);
          fflush(stdout);
          katana->moveMotorToEnc(GRIPPER_MOTOR_IDX, enc,
                                 /*wait*/true, /*tol*/100, /*timeout_ms*/10000);
          printf("[KNI] done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, true));
          fflush(stdout);
          break;
        }

        case '2': {
          int delta;
          if (!prompt_int("encoder delta (+/-)", delta)) break;
          printf("[KNI] moveMotorByEnc(5, %d, wait=true, 10 s)...\n", delta);
          fflush(stdout);
          katana->moveMotorByEnc(GRIPPER_MOTOR_IDX, delta,
                                 /*wait*/true, /*timeout_ms*/10000);
          printf("[KNI] done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, true));
          fflush(stdout);
          break;
        }

        case '5': {
          int dif;
          if (!prompt_int("encoder increment (+)", dif)) break;
          printf("[KNI] inc(5, %d, wait=true, tol=100, 10 s)...\n", dif);
          fflush(stdout);
          katana->inc(GRIPPER_MOTOR_IDX, dif, /*wait*/true, /*tol*/100, /*timeout*/10000);
          printf("[KNI] done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, true));
          fflush(stdout);
          break;
        }

        case '6': {
          int dif;
          if (!prompt_int("encoder decrement (enter positive val)", dif)) break;
          printf("[KNI] dec(5, %d, wait=true, tol=100, 10 s)...\n", dif);
          fflush(stdout);
          katana->dec(GRIPPER_MOTOR_IDX, dif, /*wait*/true, /*tol*/100, /*timeout*/10000);
          printf("[KNI] done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, true));
          fflush(stdout);
          break;
        }

        case '7': {
          int tar;
          if (!prompt_int("target encoder (abs)", tar)) break;
          printf("[KNI] mov(5, %d, wait=true, tol=100, 10 s)...\n", tar);
          fflush(stdout);
          katana->mov(GRIPPER_MOTOR_IDX, tar, /*wait*/true, /*tol*/100, /*timeout*/10000);
          printf("[KNI] done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, true));
          fflush(stdout);
          break;
        }

        // ── Move motor 6 — by angle ────────────────────────────────────────

        case '3': {
          double rad;
          if (!prompt_double("KNI radians (abs, NOT URDF)", rad)) break;
          printf("[KNI] moveMotorTo(5, %.4f rad, wait=true, 10 s)...\n", rad);
          fflush(stdout);
          katana->moveMotorTo(GRIPPER_MOTOR_IDX, rad,
                              /*wait*/true, /*timeout_ms*/10000);
          printf("[KNI] done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, true));
          fflush(stdout);
          break;
        }

        case '4': {
          double rad;
          if (!prompt_double("KNI radians delta (+/-, NOT URDF)", rad)) break;
          printf("[KNI] moveMotorBy(5, %.4f rad, wait=true, 10 s)...\n", rad);
          fflush(stdout);
          katana->moveMotorBy(GRIPPER_MOTOR_IDX, rad,
                              /*wait*/true, /*timeout_ms*/10000);
          printf("[KNI] done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, true));
          fflush(stdout);
          break;
        }

        case '8': {
          double deg;
          if (!prompt_double("KNI degrees (abs, NOT URDF)", deg)) break;
          printf("[KNI] movDegrees(5, %.2f deg, wait=true, tol=100, 10 s)...\n", deg);
          fflush(stdout);
          katana->movDegrees(GRIPPER_MOTOR_IDX, deg,
                             /*wait*/true, /*tol*/100, /*timeout*/10000);
          printf("[KNI] done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, true));
          fflush(stdout);
          break;
        }

        case '9': {
          double dif;
          if (!prompt_double("KNI degrees increment (+)", dif)) break;
          printf("[KNI] incDegrees(5, %.2f deg, wait=true, tol=100, 10 s)...\n", dif);
          fflush(stdout);
          katana->incDegrees(GRIPPER_MOTOR_IDX, dif,
                             /*wait*/true, /*tol*/100, /*timeout*/10000);
          printf("[KNI] done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, true));
          fflush(stdout);
          break;
        }

        case '0': {
          double dif;
          if (!prompt_double("KNI degrees decrement (enter positive val)", dif)) break;
          printf("[KNI] decDegrees(5, %.2f deg, wait=true, tol=100, 10 s)...\n", dif);
          fflush(stdout);
          katana->decDegrees(GRIPPER_MOTOR_IDX, dif,
                             /*wait*/true, /*tol*/100, /*timeout*/10000);
          printf("[KNI] done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, true));
          fflush(stdout);
          break;
        }

        // ── Spline — Hermite cubic, zero velocity at both ends ─────────────

        case 's': {
          // Read current encoder as start; prompt for target only.
          int startEnc = katana->getMotorEncoders(GRIPPER_MOTOR_IDX, true);
          int targetEnc;
          if (!prompt_int("spline target encoder", targetEnc)) break;

          // Hermite cubic with vs=ve=0:
          //   p1 = s
          //   p2_short = round(64 * vs*T / T)        = 0
          //   p3_short = round(1024 * 3*(e-s) / T^2)
          //   p4_short = round(32768 * -2*(e-s) / T^3)
          const double T  = static_cast<double>(SPLINE_T);
          const double ds = static_cast<double>(targetEnc - startEnc);
          const short  sp1 = static_cast<short>(startEnc);
          const short  sp2 = 0;
          const short  sp3 = static_cast<short>(std::round(1024.0 * 3.0 * ds / (T * T)));
          const short  sp4 = static_cast<short>(std::round(32768.0 * (-2.0) * ds / (T * T * T)));

          printf("[KNI] sendSplineToMotor(5, target=%d, T=%d, p1=%d, p2=%d, p3=%d, p4=%d)\n",
                 targetEnc, SPLINE_T, (int)sp1, (int)sp2, (int)sp3, (int)sp4);
          printf("[KNI] startSplineMovement(exactflag=true, moreflag=1)...\n");
          fflush(stdout);

          katana->sendSplineToMotor(
            static_cast<short>(GRIPPER_MOTOR_IDX),
            static_cast<short>(targetEnc),
            static_cast<short>(SPLINE_T),
            sp1, sp2, sp3, sp4);
          katana->startSplineMovement(true /*exactflag*/, 1 /*moreflag=last*/);

          // Wait for segment to complete (T * 10 ms + margin)
          std::this_thread::sleep_for(std::chrono::milliseconds(SPLINE_T * 10 + 300));
          printf("[KNI] spline done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, true));
          fflush(stdout);
          break;
        }

        // ── Per-motor power (motor 6 only) ────────────────────────────────

        case 'E':
          katana->switchMotorOn(static_cast<short>(GRIPPER_MOTOR_IDX));
          printf("[KNI] switchMotorOn(5) — motor 6 enabled.\n"); fflush(stdout);
          break;

        case 'X':
          katana->switchMotorOff(static_cast<short>(GRIPPER_MOTOR_IDX));
          printf("[KNI] switchMotorOff(5) — motor 6 disabled.\n"); fflush(stdout);
          break;

        case 'F':
          katana->freezeMotor(static_cast<short>(GRIPPER_MOTOR_IDX));
          printf("[KNI] freezeMotor(5) — motor 6 frozen.\n"); fflush(stdout);
          break;

        // ── Robot-wide power ──────────────────────────────────────────────

        case 'e':
          katana->switchRobotOn();
          printf("[KNI] switchRobotOn() — all motors enabled.\n"); fflush(stdout);
          break;

        case 'd':
          katana->freezeRobot();
          printf("[KNI] freezeRobot() — all motors frozen, arm limp.\n"); fflush(stdout);
          break;

        // ── Info / limits ─────────────────────────────────────────────────

        case 'r': {
          int enc = katana->getMotorEncoders(GRIPPER_MOTOR_IDX, /*refresh=*/true);
          double kni_rad = KNI_ANGLE_OFFSET + KNI_DIR * (enc / KNI_ENC_PER_CYCLE) * KNI_TWO_PI;
          double urdf    = enc_to_urdf(enc);
          printf("[KNI] getMotorEncoders(5): encoder=%-6d  kni_rad=%+.4f  urdf=%+.4f rad\n",
                 enc, kni_rad, urdf);
          fflush(stdout);
          break;
        }

        case 'v': {
          short vel  = katana->getMotorVelocityLimit(static_cast<short>(GRIPPER_MOTOR_IDX));
          short acc  = katana->getMotorAccelerationLimit(static_cast<short>(GRIPPER_MOTOR_IDX));
          int   ctrl = katana->getCurrentControllerType(GRIPPER_MOTOR_IDX);
          short frc  = katana->getForce(GRIPPER_MOTOR_IDX);
          printf("[KNI] motor 6 limits/status:\n");
          printf("  velocityLimit=%d  accelerationLimit=%d  controllerType=%d  force=%d\n",
                 (int)vel, (int)acc, ctrl, (int)frc);
          fflush(stdout);
          break;
        }

        case 'V': {
          int vel;
          if (!prompt_int("velocity limit (e.g. 100)", vel)) break;
          katana->setMotorVelocityLimit(static_cast<short>(GRIPPER_MOTOR_IDX),
                                        static_cast<short>(vel));
          printf("[KNI] setMotorVelocityLimit(5, %d) done.\n", vel); fflush(stdout);
          break;
        }

        case 'A': {
          int acc;
          if (!prompt_int("acceleration limit (e.g. 1)", acc)) break;
          katana->setMotorAccelerationLimit(static_cast<short>(GRIPPER_MOTOR_IDX),
                                            static_cast<short>(acc));
          printf("[KNI] setMotorAccelerationLimit(5, %d) done.\n", acc); fflush(stdout);
          break;
        }

        case 'w': {
          int enc;
          if (!prompt_int("target encoder to wait for", enc)) break;
          printf("[KNI] waitForMotor(5, %d, tol=100, mode=0, timeout=10 s)...\n", enc);
          fflush(stdout);
          katana->waitForMotor(static_cast<short>(GRIPPER_MOTOR_IDX),
                               enc, /*encTol*/100, /*mode*/0, /*timeout_ms*/10000);
          printf("[KNI] waitForMotor done. enc=%d\n",
                 katana->getMotorEncoders(GRIPPER_MOTOR_IDX, true));
          fflush(stdout);
          break;
        }

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
