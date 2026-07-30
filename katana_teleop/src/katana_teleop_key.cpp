/*
 * katana_teleop_key.cpp — ROS 2 / KNI low-level keyboard teleop
 *
 * Controls motor 0 (pan) and motor 1 (lift) directly via the KNI SDK.
 * Calibrates on startup, then moves the arm to a "straight up" home position
 * before entering the teleop loop.
 *
 * Usage:  ros2 run katana_teleop katana_teleop_key [<ip>] [<port>]
 *         Defaults: ip=192.168.1.1  port=5566
 *
 * Keys:
 *   W / S   motor 1 (lift)  up / down
 *   A / D   motor 0 (pan)   left / right
 *   H       return to home (straight-up) position
 *   E       enable / re-enable motors after freeze
 *   F       freeze motors (hold in place, motors stay powered)
 *   + / -   double / halve encoder step size
 *   P       print current encoder values
 *   Q       quit (freeze + power off)
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <cerrno>
#include <algorithm>
#include <memory>
#include <vector>
#include <termios.h>
#include <unistd.h>

// KNI SDK
#include "kniBase.h"
#include "KNI/cdlSocket.h"
#include "KNI/cplSerial.h"

// ament_index to locate the KNI config file at runtime
#include "ament_index_cpp/get_package_share_directory.hpp"

// ─── Constants ───────────────────────────────────────────────────────────────

static constexpr int PAN_MOTOR   = 0;   // katana_motor1_pan_joint
static constexpr int LIFT_MOTOR  = 1;   // katana_motor2_lift_joint
static constexpr int DEFAULT_STEP = 1000; // encoder ticks per keypress (~7°)

// KNI-frame radians for the "straight up" home position.
// Formula: kni_rad = urdf_rad / urdf_flip + urdf_offset  (all arm flips = -1)
//   motor0  urdf=0.0,     offset=0.8290  →  0.8290
//   motor1  urdf=0.5648,  offset=2.2731  →  1.7083
//   motor2  urdf=0.0,     offset=2.7957  →  2.7957
//   motor3  urdf=0.0,     offset=2.9095  →  2.9095
//   motor4  urdf=0.0,     offset=0.9013  →  0.9013
static constexpr double HOME_KNI_RAD[5] = {0.8290, 1.7083, 2.7957, 2.9095, 0.9013};

// ─── Global state (needed by signal handler) ─────────────────────────────────

static struct termios g_cooked;
static int g_kfd = 0;

static std::unique_ptr<CCdlSocket>    g_device;
static std::unique_ptr<CCplSerialCRC> g_protocol;
static std::unique_ptr<CLMBase>       g_katana;

// ─── Helpers ─────────────────────────────────────────────────────────────────

// Convert a KNI-frame radian angle to encoder counts for one motor.
static int radToEnc(const TMotInit* init, double kni_rad)
{
    double enc_f = (kni_rad - init->angleOffset)
                 / (init->rotationDirection * 2.0 * M_PI / init->encodersPerCycle);
    return static_cast<int>(std::round(enc_f));
}

static void printState()
{
    try {
        std::vector<int> enc = g_katana->getRobotEncoders(true);
        std::printf("  encoders: M0(yaw)=%6d  M1(lift)=%6d  M2=%6d  M3=%6d  M4=%6d  M5=%6d\n",
                    enc.size() > 0 ? enc[0] : 0,
                    enc.size() > 1 ? enc[1] : 0,
                    enc.size() > 2 ? enc[2] : 0,
                    enc.size() > 3 ? enc[3] : 0,
                    enc.size() > 4 ? enc[4] : 0,
                    enc.size() > 5 ? enc[5] : 0);

    } catch (...) {
        std::printf("  (could not read encoders)\n");
    }
    std::fflush(stdout);
}

static void restoreTerminal()
{
    tcsetattr(g_kfd, TCSANOW, &g_cooked);
}

static void cleanup()
{
    restoreTerminal();
    if (g_katana) {
        try { g_katana->freezeRobot();  } catch (...) {}
        try { g_katana->switchRobotOff(); } catch (...) {}
    }
}

static void sigHandler(int) { cleanup(); exit(0); }

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    const char* ip   = (argc > 1) ? argv[1] : "192.168.1.1";
    const int   port = (argc > 2) ? std::atoi(argv[2]) : 5566;

    // Locate config file via ament_index (works after: source install/setup.zsh)
    std::string cfg;
    try {
        cfg = ament_index_cpp::get_package_share_directory("kni")
            + "/KNI_4.3.0/configfiles400/katana6M180.cfg";
    } catch (const std::exception& e) {
        std::fprintf(stderr,
            "[teleop] Cannot locate kni share dir: %s\n"
            "         Did you source install/setup.zsh ?\n", e.what());
        return 1;
    }

    std::printf("[teleop] Connecting to %s:%d ...\n", ip, port);
    std::printf("[teleop] Config: %s\n", cfg.c_str());

    // ── Connect ──────────────────────────────────────────────────────────────
    try {
        g_device   = std::make_unique<CCdlSocket>(const_cast<char*>(ip), port);
        g_protocol = std::make_unique<CCplSerialCRC>();
        g_protocol->init(g_device.get());
        g_katana   = std::make_unique<CLMBase>();
        g_katana->create(cfg.c_str(), g_protocol.get());
        g_katana->setGripperParameters(true, 30770, 15000);
        std::printf("[teleop] Connected.\n");
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[teleop] Connection failed: %s\n", e.what());
        return 1;
    }

    signal(SIGINT,  sigHandler);
    signal(SIGTERM, sigHandler);

    // ── Calibrate ────────────────────────────────────────────────────────────
    std::printf("[teleop] Clearing fault flags...\n");
    try { g_katana->unBlock(); } catch (...) {}

    std::printf("[teleop] Calibrating (arm moves to all joint limits) ...\n");
    try {
        g_katana->calibrate();
        g_katana->unBlock();
        std::printf("[teleop] Calibration complete.\n");
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[teleop] Calibration failed: %s\n", e.what());
        cleanup();
        return 1;
    }

    // ── Compute home encoder values ───────────────────────────────────────────
    const TKatMOT* motors    = g_katana->GetBase()->GetMOT();
    const int      num_motors = motors->cnt;
    const int      arm_motors = std::min(5, num_motors);

    std::vector<int> home_enc(num_motors);
    // Seed everything at current calibrated position first
    std::vector<int> cur = g_katana->getRobotEncoders(true);
    for (int i = 0; i < num_motors; ++i)
        home_enc[i] = (i < static_cast<int>(cur.size())) ? cur[i] : 0;

    // Override the 5 arm joints with the computed straight-up targets,
    // clamped to [enc_min+200, enc_max-200] — same margin the hardware interface uses.
    // Some home positions (motor1 lift, motor3 elbow) land just outside the firmware
    // soft limits; clamping gives the closest safe encoder without an "out of range" error.
    static constexpr int kMargin = 200;
    for (int i = 0; i < arm_motors; ++i) {
        const TMotInit* init = motors->arr[i].GetInitialParameters();
        const int raw     = radToEnc(init, HOME_KNI_RAD[i]);
        const int enc_min = motors->arr[i].GetEncoderMinPos() + kMargin;
        const int enc_max = motors->arr[i].GetEncoderMaxPos() - kMargin;
        home_enc[i] = std::max(enc_min, std::min(enc_max, raw));
        if (home_enc[i] != raw)
            std::printf("[teleop]   motor%d home encoder = %d  (clamped from %d, limits [%d,%d])\n",
                        i, home_enc[i], raw, enc_min, enc_max);
        else
            std::printf("[teleop]   motor%d home encoder = %d\n", i, home_enc[i]);
    }

    // ── Move to home ─────────────────────────────────────────────────────────
    std::printf("[teleop] Moving to home (straight up) ...\n");
    try {
        g_katana->moveRobotToEnc(home_enc, /*wait=*/true, /*tol=*/100, /*timeout=*/30000);
        std::printf("[teleop] Home position reached.\n");
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[teleop] Move to home failed: %s\n", e.what());
        cleanup();
        return 1;
    }

    // ── Help text ─────────────────────────────────────────────────────────────
    std::printf("\n");
    std::printf("  +-----------------------------------------+\n");
    std::printf("  |   Katana 400 Low-Level Teleop (KNI)     |\n");
    std::printf("  +-----------------------------------------+\n");
    std::printf("  |  W / S  ->  motor1 (lift)   up / down   |\n");
    std::printf("  |  A / D  ->  motor0 (pan)  left / right  |\n");
    std::printf("  |  H      ->  go to home position          |\n");
    std::printf("  |  E      ->  enable motors                |\n");
    std::printf("  |  F      ->  freeze motors                |\n");
    std::printf("  |  + / -  ->  double / halve step size     |\n");
    std::printf("  |  P      ->  print encoder values         |\n");
    std::printf("  |  Q      ->  quit                         |\n");
    std::printf("  +-----------------------------------------+\n\n");
    std::fflush(stdout);

    // ── Raw terminal mode ─────────────────────────────────────────────────────
    struct termios raw;
    tcgetattr(g_kfd, &g_cooked);
    std::memcpy(&raw, &g_cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL]  = 1;
    raw.c_cc[VEOF]  = 2;
    raw.c_cc[VMIN]  = 1;   // block until at least 1 byte is available
    raw.c_cc[VTIME] = 0;   // no read timeout
    tcsetattr(g_kfd, TCSANOW, &raw);

    int step = DEFAULT_STEP;
    std::printf("  step = %d enc ticks\n", step);
    printState();

    // ── Keyboard loop ─────────────────────────────────────────────────────────
    char c;
    while (true) {
        ssize_t n = read(g_kfd, &c, 1);
        if (n < 0) {
            if (errno == EINTR) continue;   // interrupted by signal, retry
            perror("[teleop] read()");
            break;
        }
        if (n == 0) break;                  // EOF (stdin closed)
        try {
            switch (c) {
                case 'w': case 'W':
                    std::printf("  [lift +%d]  ", step);
                    g_katana->inc(LIFT_MOTOR, step, /*wait=*/true, /*tol=*/100);
                    printState();
                    break;

                case 's': case 'S':
                    std::printf("  [lift -%d]  ", step);
                    g_katana->dec(LIFT_MOTOR, step, /*wait=*/true, /*tol=*/100);
                    printState();
                    break;

                case 'a': case 'A':
                    std::printf("  [pan  +%d]  ", step);
                    g_katana->inc(PAN_MOTOR, step, /*wait=*/true, /*tol=*/100);
                    printState();
                    break;

                case 'd': case 'D':
                    std::printf("  [pan  -%d]  ", step);
                    g_katana->dec(PAN_MOTOR, step, /*wait=*/true, /*tol=*/100);
                    printState();
                    break;

                case 'h': case 'H':
                    std::printf("  [-> home]\n");
                    g_katana->moveRobotToEnc(home_enc, true, 100, 30000);
                    printState();
                    break;

                case 'e': case 'E':
                    std::printf("  [motors ON]\n");
                    g_katana->switchRobotOn();
                    g_katana->unBlock();
                    break;

                case 'f': case 'F':
                    std::printf("  [freeze]\n");
                    g_katana->freezeRobot();
                    break;

                case '+': case '=':
                    step *= 2;
                    std::printf("  [step -> %d]\n", step);
                    break;

                case '-': case '_':
                    step = std::max(50, step / 2);
                    std::printf("  [step -> %d]\n", step);
                    break;

                case 'p': case 'P':
                    printState();
                    break;

                case 'q': case 'Q':
                    std::printf("  [quit]\n");
                    cleanup();
                    return 0;

                default:
                    break;
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "  [KNI error] %s\n", e.what());
            std::fflush(stderr);
        }
    }

    cleanup();
    return 0;
}
