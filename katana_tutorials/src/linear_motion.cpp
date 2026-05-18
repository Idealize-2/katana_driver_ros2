// =============================================================================
// linear_motion.cpp  —  Tutorial 6
// =============================================================================
// Moves the Katana 400 6M180 TCP in a TRUE STRAIGHT LINE through Cartesian
// space using KNI's moveRobotLinearTo().  No ROS 2 or MoveIt required —
// connects directly to the arm via TCP or serial.
//
// IMPORTANT: do NOT run this while ros2_control (real_hardware.launch.py) is
// active — both processes fight for the same TCP socket.
//
// RUN:
//   ros2 run katana_tutorials linear_motion
//       tcp 192.168.1.1 /path/to/katana6M180.cfg
//
// UNITS:  X/Y/Z in mm,  phi/theta/psi in radians
//
// EXAMPLE SAFE WAYPOINTS (from KNI demo, Katana 400 6M180):
//   -20.85  -211.40  120.55   phi=-0.0983  theta=2.3570  psi=0.1821
//   -25.79  -246.24    2.22   phi=-0.1043  theta=2.6742  psi=0.1821
//   111.70  -227.38  -17.74   phi= 0.4566  theta=2.7450  psi=0.1821
//   113.31  -286.94  136.27   phi= 0.3761  theta=2.1827  psi=-0.2793
// =============================================================================

#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "kniBase.h"
#include "KNI/cdlCOM.h"
#include "KNI/cdlSocket.h"
#include "KNI/cplSerial.h"

static volatile bool g_running = true;
static void signalHandler(int) { g_running = false; }

static void printPose(const char * label,
                      double x, double y, double z,
                      double phi, double theta, double psi)
{
  std::cout << std::fixed << std::setprecision(2);
  std::cout << label
            << "  X=" << std::setw(8) << x
            << "  Y=" << std::setw(8) << y
            << "  Z=" << std::setw(8) << z
            << " mm"
            << "   phi=" << std::setw(7) << std::setprecision(4) << phi
            << "  theta=" << std::setw(7) << theta
            << "  psi=" << std::setw(7) << psi
            << " rad\n";
}

static void printPreview(double cx, double cy, double cz,
                         double tx, double ty, double tz,
                         double phi, double theta, double psi)
{
  std::cout << "\n" << std::string(68, '-') << "\n";
  std::cout << std::left << std::setw(10) << "Axis"
            << std::setw(14) << "Current (mm)"
            << std::setw(14) << "Target (mm)"
            << "Delta (mm)\n";
  std::cout << std::string(68, '-') << "\n";

  auto row = [&](const char * axis, double cur, double tgt) {
    double delta = tgt - cur;
    std::cout << std::setw(10) << axis
              << std::fixed << std::setprecision(2)
              << std::setw(14) << cur
              << std::setw(14) << tgt
              << (delta >= 0.0 ? "+" : "") << delta << "\n";
  };
  row("X", cx, tx);
  row("Y", cy, ty);
  row("Z", cz, tz);

  std::cout << "\nOrientation kept fixed:  phi=" << std::setprecision(4) << phi
            << "  theta=" << theta << "  psi=" << psi << " rad\n";
  std::cout << std::string(68, '-') << "\n";
}

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::cerr << "Usage:   linear_motion <tcp|serial> <IP_or_PortNum> <CONFIG_FILE>\n"
              << "Example: linear_motion tcp 192.168.1.1 "
              << "/path/to/kni/KNI_4.3.0/configfiles400/katana6M180.cfg\n\n"
              << "NOTE: stop ros2_control (real_hardware.launch.py) before running.\n";
    return 1;
  }

  std::string type        = argv[1];
  std::string addr        = argv[2];
  std::string config_file = argv[3];

  std::signal(SIGINT, signalHandler);

  // Declare outside the try block so destructors run after the catch.
  std::unique_ptr<CCdlBase>       device;
  std::unique_ptr<CCplSerialCRC>  protocol;
  std::unique_ptr<CLMBase>        katana;

  try {
    // ── Connect ──────────────────────────────────────────────────────────────
    if (type == "tcp") {
      device = std::make_unique<CCdlSocket>(const_cast<char *>(addr.c_str()), 5566);
      std::cout << "  [OK] TCP socket opened to " << addr << "\n";
    } else {
      TCdlCOMDesc ccd;
      ccd.port = std::stoi(addr);
      ccd.baud = 57600;
      ccd.data = 8; ccd.parity = 'N'; ccd.stop = 1; ccd.rttc = 100; ccd.wttc = 100;
      device = std::make_unique<CCdlCOM>(ccd);
      std::cout << "  [OK] Serial port /dev/ttyS" << addr << " opened\n";
    }

    protocol = std::make_unique<CCplSerialCRC>();
    protocol->init(device.get());
    std::cout << "  [OK] Protocol initialised.\n";

    katana = std::make_unique<CLMBase>();
    katana->create(config_file.c_str(), protocol.get());
    std::cout << "  [OK] Arm object created from config.\n";

    // ── Calibrate ─────────────────────────────────────────────────────────────
    std::cout << "\nCalibrate arm? (y = yes / n = skip): ";
    char choice = 'n';
    std::cin >> choice;
    std::cin.ignore(4096, '\n');

    if (choice == 'y' || choice == 'Y') {
      std::cout << "Calibrating — keep the workspace clear!\n";
      katana->calibrate();
      std::cout << "  [OK] Calibration complete.\n";
    }

    // ── Set linear velocity ───────────────────────────────────────────────────
    double velocity_mm_s = 5.0;
    katana->setMaximumLinearVelocity(velocity_mm_s);
    std::cout << "\nLinear velocity set to " << velocity_mm_s << " mm/s\n"
              << "  (type 'v <value>' at any prompt to change, e.g. 'v 10')\n";

    // ── Interactive motion loop ───────────────────────────────────────────────
    std::cout << "\nSuggested safe waypoints (from KNI demo, Katana 400 6M180):\n"
              << "  1)  X= -20.85  Y=-211.40  Z= 120.55\n"
              << "  2)  X= -25.79  Y=-246.24  Z=   2.22\n"
              << "  3)  X= 111.70  Y=-227.38  Z= -17.74\n"
              << "  4)  X= 113.31  Y=-286.94  Z= 136.27\n\n";

    while (g_running) {
      // Read current pose
      double cx, cy, cz, phi, theta, psi;
      katana->getCoordinates(cx, cy, cz, phi, theta, psi);
      std::cout << "\nCurrent pose:  ";
      printPose("", cx, cy, cz, phi, theta, psi);
      std::cout << "Velocity: " << velocity_mm_s << " mm/s\n";

      // Prompt for target XYZ (or velocity change)
      std::cout << "\nEnter  X Y Z (mm)  or  v <speed>  or Ctrl+C to quit:\n> ";
      std::string line;
      if (!std::getline(std::cin, line) || !g_running) break;

      // Velocity change command
      if (!line.empty() && line[0] == 'v') {
        double v;
        std::istringstream vs(line.substr(1));
        if (vs >> v && v > 0.0) {
          velocity_mm_s = v;
          katana->setMaximumLinearVelocity(velocity_mm_s);
          std::cout << "  Velocity updated to " << velocity_mm_s << " mm/s\n";
        } else {
          std::cerr << "  Usage: v <speed_mm_s>  e.g.  v 10\n";
        }
        continue;
      }

      double tx, ty, tz;
      std::istringstream ss(line);
      if (!(ss >> tx >> ty >> tz)) {
        std::cerr << "  Bad input — enter three numbers, e.g.:  -20.85 -211.40 120.55\n";
        continue;
      }

      // Preview
      printPreview(cx, cy, cz, tx, ty, tz, phi, theta, psi);

      std::cout << "Press Enter to move the REAL ARM, Ctrl+C to abort: ";
      std::string confirm;
      if (!std::getline(std::cin, confirm) || !g_running) break;

      // Execute linear move — catch per-move errors so the loop continues
      std::cout << "Moving...\n";
      try {
        katana->moveRobotLinearTo(tx, ty, tz, phi, theta, psi);
      } catch (const Exception & e) {
        std::cerr << "  [KNI ERROR] " << e.message() << "\n";
        std::cerr << "  Tip: if 'joint speed too high', try a lower velocity"
                  << " (type 'v 3') or a closer target.\n";
        continue;
      }

      // Report final position
      double nx, ny, nz, np, nt, ns;
      katana->getCoordinates(nx, ny, nz, np, nt, ns);
      std::cout << "Reached pose:  ";
      printPose("", nx, ny, nz, np, nt, ns);

      std::cout << "\nMove again? [y/N]: ";
      std::string again;
      if (!std::getline(std::cin, again) || again.empty() ||
          (again[0] != 'y' && again[0] != 'Y')) {
        break;
      }
    }

  } catch (const Exception & e) {
    std::cerr << "\n[KNI ERROR] " << e.message() << "\n";
    return 1;
  } catch (const std::exception & e) {
    std::cerr << "\n[ERROR] " << e.what() << "\n";
    return 1;
  }

  std::cout << "\nDone.\n";
  return 0;
}
