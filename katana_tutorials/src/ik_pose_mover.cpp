// =============================================================================
// ik_pose_mover.cpp  —  Tutorial 4: Calibrate → Limp & observe → IK check → Move
// =============================================================================
//
// Guided 4-step workflow for moving the arm to a Cartesian target:
//
//   Step 0  Calibrate the arm (moves joints to all limits and back to find zero)
//   Step 1  Disable motors — arm goes limp so you can move it by hand.
//           Current TCP pose is printed continuously. Press Enter when done.
//   Step 2  Enter target X Y Z Al Be Ga solution_iter.
//           IKCalculate() validates reachability — if unreachable you are asked
//           to enter a different target. If reachable the calculated joint
//           encoder solution is printed. Press Enter to confirm the move.
//   Step 3  Re-enable motors and moveRobotTo() the confirmed target.
//           Final pose is printed on arrival.
//
// UNITS:
//   X, Y, Z      — mm (millimetres)
//   Al, Be, Ga   — rad (Euler ZYX: phi, theta, psi)
//   solution_iter — integer index of the IK solution variant to prefer
//                   (0 = solution closest to current config; usually correct)
//
// IMPORTANT:
//   Stop ros2_control (real_hardware.launch.py) before running — both
//   processes fight over the same TCP socket.
//
// RUN (all args optional — defaults to tcp 192.168.1.1 and automatic config resolution):
//   ros2 run katana_tutorials ik_pose_mover
// =============================================================================

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// ament_index — resolves the kni share dir at runtime
#include "ament_index_cpp/get_package_share_directory.hpp"

// KNI SDK
#include "kniBase.h"
#include "KNI/cdlSocket.h"
#include "KNI/cdlCOM.h"
#include "KNI/cplSerial.h"
#include "KNI_InvKin/KatanaKinematics.h"   // NoSolutionException

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void printPose(const char * label,
                      double x, double y, double z,
                      double al, double be, double ga)
{
  std::cout << std::fixed << std::setprecision(2);
  std::cout << label
            << "  X=" << std::setw(9) << x   << " mm"
            << "  Y=" << std::setw(9) << y   << " mm"
            << "  Z=" << std::setw(9) << z   << " mm"
            << "   Al=" << std::setw(8) << std::setprecision(4) << al  << " rad"
            << "  Be=" << std::setw(8) << be  << " rad"
            << "  Ga=" << std::setw(8) << ga  << " rad"
            << "\n";
}

static std::string defaultConfigFile()
{
  try {
    return ament_index_cpp::get_package_share_directory("kni")
           + "/KNI_4.3.0/configfiles400/katana6M180.cfg";
  } catch (...) {
    return "";
  }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  // ── Parse args with defaults ───────────────────────────────────────────────
  const std::string conn_type   = (argc > 1) ? argv[1] : "tcp";
  const std::string addr        = (argc > 2) ? argv[2] : "192.168.1.1";
  const std::string config_file = (argc > 3) ? argv[3] : defaultConfigFile();

  if (config_file.empty()) {
    std::cerr
      << "Usage:   ik_pose_mover [tcp|serial] [IP_or_PortNum] [CONFIG_FILE]\n"
      << "Example: ik_pose_mover tcp 192.168.1.1 "
      << "/path/to/kni/KNI_4.3.0/configfiles400/katana6M180.cfg\n\n"
      << "ERROR: Could not locate fallback kni config file. Did you source install/setup.bash?\n";
    return 1;
  }

  std::cout << "[ik_pose_mover] conn_type=" << conn_type
            << "  addr=" << addr
            << "\n[ik_pose_mover] cfg=" << config_file << "\n\n";

  // Declare outside try so destructors run after catch.
  std::unique_ptr<CCdlBase>       device;
  std::unique_ptr<CCplSerialCRC>  protocol;
  std::unique_ptr<CLMBase>        katana;

  try {
    // ── Connect ───────────────────────────────────────────���──────────────────
    if (conn_type == "tcp") {
      device = std::make_unique<CCdlSocket>(const_cast<char *>(addr.c_str()), 5566);
      std::cout << "  [OK] TCP socket opened to " << addr << "\n";
    } else if (conn_type == "serial") {
      TCdlCOMDesc ccd;
      ccd.port = std::stoi(addr);
      ccd.baud = 57600;
      ccd.data = 8; ccd.parity = 'N'; ccd.stop = 1; ccd.rttc = 100; ccd.wttc = 100;
      device = std::make_unique<CCdlCOM>(ccd);
      std::cout << "  [OK] Serial port /dev/ttyS" << addr << " opened\n";
    } else {
      std::cerr << "Unknown connection type '" << conn_type
                << "'. Use 'tcp' or 'serial'.\n";
      return 1;
    }

    protocol = std::make_unique<CCplSerialCRC>();
    protocol->init(device.get());
    std::cout << "  [OK] Protocol initialised.\n";

    katana = std::make_unique<CLMBase>();
    katana->create(config_file.c_str(), protocol.get());
    std::cout << "  [OK] Arm object created from config.\n\n";

    const int numMotors = katana->GetBase()->GetMOT()->cnt;

    // ── STEP 0: Calibrate ──────────────────────────────────────────���──────────
    std::cout << "╔══════════════════════════════════════════════════╗\n"
              << "║  STEP 0 — Calibrate                             ║\n"
              << "╚═══════════════���══════════════════════���═══════════╝\n";
    std::cout << "Calibrating arm — keep the workspace clear!\n";
    katana->calibrate();
    katana->unBlock();
    std::cout << "  [OK] Calibration complete.\n\n";

    // ── STEP 1: Disable motors + print live pose ───────────────��──────────────
    std::cout << "╔══════════════════════════════════════���═══════════╗\n"
              << "║  STEP 1 — Arm limp: move it by hand             ║\n"
              << "╚═════════════���═══════════════════════════��════════╝\n";
    std::cout << "Disabling motors...\n";
    katana->switchRobotOff();
    std::cout << "Arm is LIMP — you can now move it freely.\n"
              << "Current TCP pose is shown below (refreshes ~5 Hz).\n"
              << "Press Enter when the arm is in the desired starting pose.\n\n";

    // Background thread: waits for Enter, sets flag.
    std::atomic<bool> enter_pressed{false};
    std::thread input_thread([&enter_pressed]() {
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      enter_pressed = true;
    });

    while (!enter_pressed.load()) {
      double cx, cy, cz, cal, cbe, cga;
      try {
        katana->getCoordinates(cx, cy, cz, cal, cbe, cga);
        // Print on same line, overwrite with \r
        std::cout << "\r"
                  << std::fixed << std::setprecision(2)
                  << "X=" << std::setw(9) << cx   << " mm  "
                  << "Y=" << std::setw(9) << cy   << " mm  "
                  << "Z=" << std::setw(9) << cz   << " mm    "
                  << std::setprecision(4)
                  << "Al=" << std::setw(8) << cal  << "  "
                  << "Be=" << std::setw(8) << cbe  << "  "
                  << "Ga=" << std::setw(8) << cga  << " rad  "
                  << std::flush;
      } catch (const Exception & e) {
        std::cout << "\r[read error: " << e.message() << "]          " << std::flush;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    input_thread.join();
    std::cout << "\n\n";

    // Re-enable motors so IKCalculate (which reads current encoders) works.
    std::cout << "Re-enabling motors...\n";
    katana->switchRobotOn();
    katana->unBlock();
    std::cout << "  [OK] Motors on.\n\n";

    // ── STEP 2 / 3: IK check + move loop ────────────────────────────────────
    double tx = 0, ty = 0, tz = 0, tal = 0, tbe = 0, tga = 0;
    int    solution_iter = 0;
    std::vector<int> ik_solution(numMotors, 0);

    while (true) {
      // ── Step 2: enter target + validate IK ──────────────────────────────
      std::cout << "╔════════════════════════════════════════════════╗\n"
                << "║  STEP 2 — Enter target pose + IK validation   ║\n"
                << "╚════════════════════════════════════════════════╝\n";
      std::cout << "Enter target pose (mm for X/Y/Z, rad for Al/Be/Ga, int for solution_iter).\n"
                << "  solution_iter: 0 = solution nearest to current config (recommended)\n\n";

      // IK input sub-loop — re-prompt until a reachable target is entered.
      while (true) {
        std::cout << "Enter X Y Z Al Be Ga solution_iter: ";
        if (!(std::cin >> tx >> ty >> tz >> tal >> tbe >> tga >> solution_iter)) {
          std::cerr << "  [!] Invalid input — please enter 7 numbers.\n\n";
          std::cin.clear();
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          continue;
        }
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        std::cout << "  Checking IK for ("
                  << std::fixed << std::setprecision(2)
                  << tx << ", " << ty << ", " << tz << ") mm  "
                  << std::setprecision(4)
                  << "Al=" << tal << " Be=" << tbe << " Ga=" << tga
                  << " rad  solution_iter=" << solution_iter << "  ...\n";

        bool reachable = false;
        try {
          katana->IKCalculate(tx, ty, tz, tal, tbe, tga,
                              ik_solution.begin() + solution_iter);
          reachable = true;
        } catch (const KNI::NoSolutionException &) {
          reachable = false;
        } catch (const Exception & e) {
          std::cerr << "  [KNI error during IK] " << e.message() << "\n";
          reachable = false;
        }

        if (!reachable) {
          std::cout << "\n  ✗  Target is NOT reachable (no IK solution found).\n"
                    << "     Try a closer target or a different orientation.\n\n";
          continue;
        }

        std::cout << "\n  ✓  Target is REACHABLE.\n";
        std::cout << "     IK solution (encoder values per motor):\n";
        for (int i = 0; i < numMotors; ++i) {
          std::cout << "       Motor " << (i + 1) << ": " << ik_solution[i] << "\n";
        }
        std::cout << "\n";

        std::cout << "Press Enter to move the REAL ARM to this pose, Ctrl+C to abort: "
                  << std::flush;
        std::string confirm;
        std::getline(std::cin, confirm);
        break;   // target confirmed — move to step 3
      }

      // ── Step 3: execute move ─────────────────────────────────────────────
      std::cout << "╔════════════════════════════════════════════════╗\n"
                << "║  STEP 3 — Execute move                        ║\n"
                << "╚════════════════════════════════════════════════╝\n";

      double bx, by, bz, bal, bbe, bga;
      katana->getCoordinates(bx, by, bz, bal, bbe, bga);
      printPose("Before:", bx, by, bz, bal, bbe, bga);
      printPose("Target:", tx, ty, tz, tal, tbe, tga);

      std::cout << "\nMoving...\n";
      katana->moveRobotTo(tx, ty, tz, tal, tbe, tga, /*waitUntilReached=*/true);

      double fx, fy, fz, fal, fbe, fga;
      katana->getCoordinates(fx, fy, fz, fal, fbe, fga);
      std::cout << "\n";
      printPose("Reached:", fx, fy, fz, fal, fbe, fga);
      std::cout << "\n  [OK] Move complete.\n";

      // ── Post-move menu ───────────────────────────────────────────────────
      std::cout << "\n"
                << "  [n]  Enter a new target  (back to Step 2)\n"
                << "  [q]  Quit\n"
                << "\nChoice: " << std::flush;
      std::string choice;
      std::getline(std::cin, choice);
      if (choice.empty() || choice[0] == 'q' || choice[0] == 'Q') {
        std::cout << "Exiting.\n";
        break;
      }
      // Any other key → loop back to Step 2
      std::cout << "\n";
    }

  } catch (const Exception & e) {
    std::cerr << "\n[KNI ERROR] " << e.message() << "\n";
    return 1;
  } catch (const std::exception & e) {
    std::cerr << "\n[ERROR] " << e.what() << "\n";
    return 1;
  }

  return 0;
}
