/*
 * arm_disable_encoder.cpp
 *
 * Connects to the Katana arm, switches motors OFF (arm goes limp so it can
 * be moved by hand), then continuously prints encoder positions until Ctrl+C.
 * Useful after a crash or to manually reposition joints.
 *
 * Usage (all args optional — defaults shown):
 *   ros2 run katana_test arm_disable_encoder
 *   ros2 run katana_test arm_disable_encoder tcp 192.168.1.1
 *   ros2 run katana_test arm_disable_encoder tcp 192.168.1.1 /full/path/to/katana6M180.cfg
 *   ros2 run katana_test arm_disable_encoder serial 0      # /dev/ttyS0, 57600
 *
 * Defaults:
 *   type        = tcp
 *   ip/port     = 192.168.1.1  (tcp) | 0  (serial)
 *   config_file = <kni_share>/KNI_4.3.0/configfiles400/katana6M180.cfg
 */

#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>
#include <csignal>
#include <cmath>
#include <thread>
#include <chrono>

// KNI SDK
#include "kniBase.h"
#include "KNI/cdlSocket.h"
#include "KNI/cdlCOM.h"
#include "KNI/cplSerial.h"

// ament_index — resolves the kni share dir at runtime
#include "ament_index_cpp/get_package_share_directory.hpp"

static volatile bool g_running = true;
static void signalHandler(int) { g_running = false; }

static double encoderToDegrees(const TMotInit * init, int encoder)
{
  if (init->encodersPerCycle == 0) return 0.0;
  double rad = init->angleOffset
             + init->rotationDirection
             * (static_cast<double>(encoder) / init->encodersPerCycle)
             * (2.0 * M_PI);
  return rad * (180.0 / M_PI);
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

int main(int argc, char ** argv)
{
  // ── Parse args with defaults ───────────────────────────────────────────────
  std::string type        = (argc > 1) ? argv[1] : "tcp";
  std::string addr        = (argc > 2) ? argv[2] : "192.168.1.1";
  std::string config_file = (argc > 3) ? argv[3] : defaultConfigFile();

  if (config_file.empty()) {
    std::cerr << "[arm_disable_encoder] ERROR: could not locate kni config file.\n"
              << "  Did you source install/setup.bash?\n"
              << "  Or pass the path explicitly as the 3rd argument.\n";
    return 1;
  }

  std::cout << "[arm_disable_encoder] type=" << type
            << "  addr=" << addr
            << "\n[arm_disable_encoder] cfg=" << config_file << "\n";

  std::signal(SIGINT, signalHandler);

  std::unique_ptr<CCdlBase>      device;
  std::unique_ptr<CCplSerialCRC> protocol;
  std::unique_ptr<CLMBase>       katana;

  try {
    if (type == "tcp") {
      device = std::make_unique<CCdlSocket>(const_cast<char*>(addr.c_str()), 5566);
      std::cout << "  [OK] TCP socket opened to " << addr << "\n";
    } else {
      TCdlCOMDesc ccd;
      ccd.port  = std::stoi(addr);
      ccd.baud  = 57600;
      ccd.data  = 8; ccd.parity = 'N'; ccd.stop = 1;
      ccd.rttc  = 100; ccd.wttc = 100;
      device = std::make_unique<CCdlCOM>(ccd);
      std::cout << "  [OK] Serial port /dev/ttyS" << addr << " opened\n";
    }

    protocol = std::make_unique<CCplSerialCRC>();
    protocol->init(device.get());

    katana = std::make_unique<CLMBase>();
    katana->create(config_file.c_str(), protocol.get());
    std::cout << "  [OK] Arm connected.\n";

    std::cout << "\nDisabling motors (arm goes limp — safe to move by hand)...\n";
    katana->switchRobotOff();
    std::cout << "Motors DISABLED. Reading encoders. Press Ctrl+C to quit.\n";
    std::cout << std::string(80, '-') << "\n";

    const TKatMOT * motors = katana->GetBase()->GetMOT();
    int n_motors = motors->cnt;

    while (g_running) {
      std::vector<int> encoders = katana->getRobotEncoders(true);
      std::cout << "Enc: ";
      for (int i = 0; i < n_motors && i < static_cast<int>(encoders.size()); ++i) {
        const TMotInit * init = motors->arr[i].GetInitialParameters();
        double deg = encoderToDegrees(init, encoders[i]);
        std::cout << "M" << i + 1 << ":"
                  << std::setw(7) << encoders[i]
                  << "(" << std::fixed << std::setprecision(1) << deg << "°) ";
      }
      std::cout << "\r" << std::flush;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
