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

static volatile bool g_running = true;
static void signalHandler(int) { g_running = false; }

double encoderToDegrees(const TMotInit * init, int encoder)
{
  if (init->encodersPerCycle == 0) return 0.0;
  double rad = init->angleOffset + init->rotationDirection * (static_cast<double>(encoder) / init->encodersPerCycle) * (2.0 * M_PI);
  return rad * (180.0 / M_PI);
}

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::cerr << "Usage: encoder_reader <tcp|serial> <IP_or_PortNum> <CONFIG_FILE>\n";
    std::cerr << "Examples:\n";
    std::cerr << "  encoder_reader tcp 192.168.1.1 /path/to/arm.cfg\n";
    std::cerr << "  encoder_reader serial 0 /path/to/arm.cfg  (0 = /dev/ttyS0)\n";
    return 1;
  }

  std::string type = argv[1];
  std::string addr = argv[2];
  std::string config_file = argv[3];

  std::signal(SIGINT, signalHandler);

  try {
    std::unique_ptr<CCdlBase> device;
    if (type == "tcp") {
      device = std::make_unique<CCdlSocket>(const_cast<char*>(addr.c_str()), 5566);
      std::cout << "Opened TCP socket to " << addr << "\n";
    } else {
      TCdlCOMDesc ccd;
      ccd.port = std::stoi(addr);
      ccd.baud = 57600;
      ccd.data = 8; ccd.parity = 'N'; ccd.stop = 1; ccd.rttc = 100; ccd.wttc = 100;
      device = std::make_unique<CCdlCOM>(ccd);
      std::cout << "Opened Serial port /dev/ttyS" << addr << "\n";
    }

    std::unique_ptr<CCplSerialCRC> protocol = std::make_unique<CCplSerialCRC>();
    protocol->init(device.get());

    std::unique_ptr<CLMBase> katana = std::make_unique<CLMBase>();
    katana->create(config_file.c_str(), protocol.get());
    std::cout << "Arm initialized.\n";

    const TKatMOT * motors = katana->GetBase()->GetMOT();
    int n_motors = motors->cnt;

    std::cout << std::string(80, '-') << "\n";
    std::cout << std::setw(6) << "Cycle";
    for (int i = 0; i < n_motors; ++i) {
      std::cout << " | " << std::setw(10) << ("M" + std::to_string(i+1) + " enc")
                << std::setw(8) << ("M" + std::to_string(i+1) + " deg");
    }
    std::cout << "\n" << std::string(80, '-') << "\n";

    int cycle = 0;
    while (g_running) {
      std::vector<int> encoders = katana->getRobotEncoders(true);
      std::cout << std::setw(6) << ++cycle;
      for (int i = 0; i < n_motors && i < static_cast<int>(encoders.size()); ++i) {
        const TMotInit * init = motors->arr[i].GetInitialParameters();
        double deg = encoderToDegrees(init, encoders[i]);
        std::cout << " | " << std::setw(10) << encoders[i]
                  << std::setw(8) << std::fixed << std::setprecision(2) << deg;
      }
      std::cout << "\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  } catch (const Exception & e) {
    std::cerr << "KNI Error: " << e.message() << "\n";
    return 1;
  }

  return 0;
}
