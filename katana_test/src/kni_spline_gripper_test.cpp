// kni_spline_gripper_test.cpp
//
// Direct-KNI keyboard teleop for the Katana 400 gripper using moveMotorToEnc().
//
// Background: the Katana 400 firmware ignores sendSplineToMotor() for motor 6
// (the gripper) — it has its own TPS (trapezoidal-profile) controller that is
// only accessible via moveMotorToEnc().  Sending splines to motor 6 has no
// effect, which is why ros2_control's gripper never moved with the old spline-
// only kni_loop.  This tool uses the correct API and mirrors what the updated
// kni_loop now does.
//
// !! Cannot run simultaneously with ros2_control_node — both fight over TCP. !!
//
// Parameters (--ros-args -p <name>:=<value>):
//   ip_address    arm IP              (default 192.168.1.1)
//   tcp_port      KNI port            (default 5566)
//   config_file   absolute path to .cfg  (default: auto from kni share)
//   calibrate     run calibration     (default false)
//   spline_t      segment length, KNI time units = 10 ms each (default 80 = 800 ms)
//   jog_step      initial encoder jog step (default 1000 enc units)
//
// Keyboard:
//   o   open       (enc ≈ 30500,  URDF ≈ +0.30 rad)
//   c   close      (enc ≈ 23257,  URDF ≈  0.00 rad)
//   f   full-close (enc ≈ 12240,  URDF ≈ -0.44 rad)  ← hard limit
//   j   jog +step  (increase encoder → toward open)
//   k   jog -step  (decrease encoder → toward close)
//   [   halve jog step
//   ]   double jog step
//   r   read encoder + URDF rad
//   e   enable  motors (switchRobotOn)
//   d   disable motors (freezeRobot)
//   ?   help
//   q   quit

#include <rclcpp/rclcpp.hpp>
#include "kniBase.h"
#include "KNI/cdlSocket.h"
#include "KNI/cplSerial.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ── constants ─────────────────────────────────────────────────────────────────

static constexpr int GRIPPER_IDX = 5;   // 0-based motor index for gripper

// KNI motor-6 parameters from katana6M180.cfg
static constexpr double KNI_ANGLE_OFFSET  = -2.1503;
static constexpr double KNI_DIR           =  1.0;
static constexpr double KNI_ENC_PER_CYCLE =  51200.0;
static constexpr double KNI_TWO_PI        =  6.283185307179586;

// Gear-corrected URDF mapping (offset=0.7040, flip=0.3255)
static constexpr double URDF_OFFSET = 0.7040;
static constexpr double URDF_FLIP   = 0.3255;

// Named gripper positions (encoder counts)
static constexpr int ENC_OPEN       = 30500;   // URDF ≈ +0.30 rad
static constexpr int ENC_HALF_CLOSE = 23257;   // URDF ≈  0.00 rad
static constexpr int ENC_FULL_CLOSE = 12240;   // URDF ≈ -0.44 rad (hard limit)

// ── helpers ───────────────────────────────────────────────────────────────────

static double enc_to_urdf(int enc)
{
  double kni_rad = KNI_ANGLE_OFFSET
                   + KNI_DIR * (static_cast<double>(enc) / KNI_ENC_PER_CYCLE) * KNI_TWO_PI;
  return (kni_rad - URDF_OFFSET) * URDF_FLIP;
}

// ── terminal helpers ──────────────────────────────────────────────────────────

static struct termios g_orig;

static void restore_terminal() { tcsetattr(STDIN_FILENO, TCSANOW, &g_orig); }

static void set_raw_mode()
{
  tcgetattr(STDIN_FILENO, &g_orig);
  atexit(restore_terminal);
  struct termios raw = g_orig;
  raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  raw.c_cc[VMIN]  = 0;
  raw.c_cc[VTIME] = 1;   // 100 ms read timeout
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

static void print_help(int step, int spline_t)
{
  printf("\n Katana gripper — sendSplineToMotor teleop\n");
  printf(" ─────────────────────────────────────────────────\n");
  printf("  o   open       (enc≈%d,  URDF≈+0.30 rad)\n", ENC_OPEN);
  printf("  c   close      (enc≈%d,  URDF≈ 0.00 rad)\n", ENC_HALF_CLOSE);
  printf("  f   full-close (enc≈%d,  URDF≈-0.44 rad)  ← hard limit\n", ENC_FULL_CLOSE);
  printf("  j   jog +%-5d (toward open)\n", step);
  printf("  k   jog -%-5d (toward close)\n", step);
  printf("  [   halve step    (now %d)\n", step);
  printf("  ]   double step   (now %d)\n", step);
  printf("  r   read encoder\n");
  printf("  e   enable  motors\n");
  printf("  d   disable motors\n");
  printf("  ?   this help\n");
  printf("  q   quit\n");
  printf("\n  spline_t = %d  (%d ms / segment)\n\n", spline_t, spline_t * 10);
  fflush(stdout);
}

// ── gripper move ──────────────────────────────────────────────────────────────
//
// Move gripper motor (5) to target_enc using TPS (moveMotorToEnc).
// The Katana 400 firmware ignores sendSplineToMotor for motor 6 — this is
// the only command that actually moves it.
//
// wait_ms: if true (default), block until the motor reaches target_enc (or
// timeout_ms elapses) so the result read-back shows the final position.

static bool move_gripper(
  CLMBase & arm,
  int target_enc,
  bool wait_ms   = true,
  int  timeout_ms = 5000)
{
  try {
    arm.moveMotorToEnc(
      static_cast<short>(GRIPPER_IDX),
      target_enc,
      /*waitUntilReached=*/wait_ms,
      /*encTolerance=*/100,
      /*waitTimeout=*/timeout_ms);
    return true;
  } catch (const std::exception & ex) {
    restore_terminal();
    printf("\n[MOVE ERROR] %s\n", ex.what());
    set_raw_mode();
    fflush(stdout);
    return false;
  }
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("kni_spline_gripper_test");

  // ── parameters ──────────────────────────────────────────────────────────────
  node->declare_parameter("ip_address",  std::string("192.168.1.1"));
  node->declare_parameter("tcp_port",    5566);
  node->declare_parameter("config_file", std::string(""));
  node->declare_parameter("calibrate",   false);
  node->declare_parameter("jog_step",    1000);  // encoder counts

  const std::string ip   = node->get_parameter("ip_address").as_string();
  const int         port = static_cast<int>(node->get_parameter("tcp_port").as_int());
  const bool        cal  = node->get_parameter("calibrate").as_bool();
  int               jog_step  = static_cast<int>(node->get_parameter("jog_step").as_int());

  std::string cfg = node->get_parameter("config_file").as_string();
  if (cfg.empty()) {
    try {
      cfg = ament_index_cpp::get_package_share_directory("kni")
            + "/KNI_4.3.0/configfiles400/katana6M180.cfg";
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(node->get_logger(),
        "Cannot resolve kni share dir: %s\n"
        "Pass config_file:=/absolute/path/to/katana6M180.cfg", ex.what());
      return 1;
    }
  }

  // ── KNI connect ─────────────────────────────────────────────────────────────
  std::unique_ptr<CCdlSocket>    device;
  std::unique_ptr<CCplSerialCRC> protocol;
  std::unique_ptr<CLMBase>       katana;

  RCLCPP_INFO(node->get_logger(),
    "[SPLINE] connecting %s:%d  cfg: %s  calibrate: %s  spline_t: %d (%d ms)",
    ip.c_str(), port, cfg.c_str(),
    cal ? "yes" : "no", spline_t, spline_t * 10);

  try {
    device   = std::make_unique<CCdlSocket>(const_cast<char *>(ip.c_str()), port);
    protocol = std::make_unique<CCplSerialCRC>();
    protocol->init(device.get());
    katana   = std::make_unique<CLMBase>();
    katana->create(cfg.c_str(), protocol.get());
    RCLCPP_INFO(node->get_logger(), "[SPLINE] connected.");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "[SPLINE] connection failed: %s", ex.what());
    return 1;
  }

  if (cal) {
    try {
      RCLCPP_INFO(node->get_logger(), "[SPLINE] calibrating...");
      katana->calibrate();
      katana->unBlock();
      RCLCPP_INFO(node->get_logger(), "[SPLINE] calibration done.");
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(node->get_logger(), "[SPLINE] calibration error: %s", ex.what());
    }
  }

  // Cache firmware encoder limits for the gripper
  const TKatMOT * motors = katana->GetBase()->GetMOT();
  const int mc = motors->cnt;   // = 6 for Katana 400 6M180
  const int enc_min = motors->arr[GRIPPER_IDX].GetEncoderMinPos() + 200;
  const int enc_max = motors->arr[GRIPPER_IDX].GetEncoderMaxPos() - 200;

  printf("[SPLINE] gripper encoder limits: min=%d  max=%d\n", enc_min, enc_max);
  fflush(stdout);

  // ── keyboard loop ────────────────────────────────────────────────────────────
  print_help(jog_step, spline_t);
  set_raw_mode();

  bool quit = false;
  while (rclcpp::ok() && !quit) {
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) <= 0) continue;

    // Read ALL current encoders before every spline send
    std::vector<int> enc;
    try {
      enc = katana->getRobotEncoders(true);
    } catch (const std::exception & ex) {
      restore_terminal();
      printf("\n[READ ERROR] %s\n", ex.what());
      set_raw_mode();
      fflush(stdout);
      continue;
    }
    // Pad to at least mc entries
    while (static_cast<int>(enc.size()) < mc) enc.push_back(0);

    int cur_gripper = enc[GRIPPER_IDX];
    int target      = cur_gripper;   // default: hold

    switch (c) {
      case 'o':
        target = ENC_OPEN;
        restore_terminal();
        printf("\n[SPLINE] → OPEN  target enc=%d  URDF≈%+.3f rad\n",
               target, enc_to_urdf(target));
        set_raw_mode(); fflush(stdout);
        break;

      case 'c':
        target = ENC_HALF_CLOSE;
        restore_terminal();
        printf("\n[SPLINE] → CLOSE  target enc=%d  URDF≈%+.3f rad\n",
               target, enc_to_urdf(target));
        set_raw_mode(); fflush(stdout);
        break;

      case 'f':
        target = ENC_FULL_CLOSE;
        restore_terminal();
        printf("\n[SPLINE] → FULL-CLOSE  target enc=%d  URDF≈%+.3f rad\n",
               target, enc_to_urdf(target));
        set_raw_mode(); fflush(stdout);
        break;

      case 'j':   // jog toward open (encoder increases)
        target = cur_gripper + jog_step;
        restore_terminal();
        printf("\n[SPLINE] jog +%d  from=%d  target=%d  URDF≈%+.3f rad\n",
               jog_step, cur_gripper, target, enc_to_urdf(target));
        set_raw_mode(); fflush(stdout);
        break;

      case 'k':   // jog toward close (encoder decreases)
        target = cur_gripper - jog_step;
        restore_terminal();
        printf("\n[SPLINE] jog -%d  from=%d  target=%d  URDF≈%+.3f rad\n",
               jog_step, cur_gripper, target, enc_to_urdf(target));
        set_raw_mode(); fflush(stdout);
        break;

      case '[':
        jog_step = std::max(50, jog_step / 2);
        restore_terminal();
        printf("\n[SPLINE] step → %d enc\n", jog_step);
        set_raw_mode(); fflush(stdout);
        continue;   // no spline to send

      case ']':
        jog_step = std::min(5000, jog_step * 2);
        restore_terminal();
        printf("\n[SPLINE] step → %d enc\n", jog_step);
        set_raw_mode(); fflush(stdout);
        continue;

      case 'r': {
        double urdf = enc_to_urdf(cur_gripper);
        restore_terminal();
        printf("\n[READ]  gripper enc=%-6d  URDF=%+.4f rad\n", cur_gripper, urdf);
        printf("        full motor state (0-based):\n");
        for (int i = 0; i < mc && i < static_cast<int>(enc.size()); ++i)
          printf("          motor %d  enc=%d\n", i, enc[i]);
        set_raw_mode(); fflush(stdout);
        continue;
      }

      case 'e':
        try {
          katana->switchRobotOn();
          restore_terminal();
          printf("\n[SPLINE] motors enabled.\n");
          set_raw_mode(); fflush(stdout);
        } catch (const std::exception & ex) {
          restore_terminal();
          printf("\n[ERROR] %s\n", ex.what());
          set_raw_mode(); fflush(stdout);
        }
        continue;

      case 'd':
        try {
          katana->freezeRobot();
          restore_terminal();
          printf("\n[SPLINE] motors frozen — arm limp.\n");
          set_raw_mode(); fflush(stdout);
        } catch (const std::exception & ex) {
          restore_terminal();
          printf("\n[ERROR] %s\n", ex.what());
          set_raw_mode(); fflush(stdout);
        }
        continue;

      case '?':
        restore_terminal();
        print_help(jog_step, spline_t);
        set_raw_mode();
        continue;

      case 'q': case '\x03':
        quit = true;
        continue;

      default:
        continue;
    }

    // Clamp target to firmware limits
    target = std::max(enc_min, std::min(enc_max, target));

    // Send: hold splines for arm (0–4) + gripper spline (5), moreflag=1
    bool ok = send_spline(*katana, enc, cur_gripper, target, spline_t, /*wait=*/true);

    if (ok) {
      // Read back actual encoder after segment completes
      try {
        int actual = katana->getMotorEncoders(GRIPPER_IDX, /*refresh=*/true);
        restore_terminal();
        printf("[RESULT] gripper enc=%-6d  URDF=%+.4f rad  (target enc=%d  URDF=%+.4f)\n",
               actual, enc_to_urdf(actual), target, enc_to_urdf(target));
        set_raw_mode(); fflush(stdout);
      } catch (const std::exception & ex) {
        restore_terminal();
        printf("[READ-BACK ERROR] %s\n", ex.what());
        set_raw_mode(); fflush(stdout);
      }
    }
  }

  // ── cleanup ──────────────────────────────────────────────────────────────────
  restore_terminal();
  printf("\n[SPLINE] shutting down...\n");
  try { katana->freezeRobot();    } catch (...) {}
  try { katana->switchRobotOff(); } catch (...) {}
  katana.reset();
  protocol.reset();
  device.reset();
  rclcpp::shutdown();
  return 0;
}
