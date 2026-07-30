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
static void signalHandler(int) { 
  std::cout << "\n[encoder_reader] Stopping...\n";
  g_running = false; 
}

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
    return 1;
  }

  std::string type = argv[1];
  std::string addr = argv[2];
  std::string config_file = argv[3];

  std::signal(SIGINT, signalHandler);

  // DECLARE THESE OUTSIDE THE TRY BLOCK so they don't get destroyed early!
  std::unique_ptr<CCdlBase> device;
  std::unique_ptr<CCplSerialCRC> protocol;
  std::unique_ptr<CLMBase> katana;

  try {
    if (type == "tcp") {
      device = std::make_unique<CCdlSocket>(const_cast<char*>(addr.c_str()), 5566);
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
    std::cout << "  [OK] Arm object created.\n";

    // --- Calibration ---
    std::cout << "\nCalibrate arm now? (y = yes / n = skip): ";
    char choice = 'n';
    std::cin >> choice;

    if (choice == 'y' || choice == 'Y') {
      std::cout << "Calibrating — keep the workspace clear!\n";
      katana->calibrate();
      std::cout << "  [OK] Calibration complete.\n";
    }

    const TKatMOT * motors = katana->GetBase()->GetMOT();
    int n_motors = motors->cnt;

    std::cout << "\nReading encoders. Press Ctrl+C to stop.\n";
    std::cout << std::string(80, '-') << "\n";
    
    while (g_running) {
      std::vector<int> encoders = katana->getRobotEncoders(true);
      std::cout << "Data: ";
      for (int i = 0; i < n_motors && i < static_cast<int>(encoders.size()); ++i) {
        const TMotInit * init = motors->arr[i].GetInitialParameters();
        double deg = encoderToDegrees(init, encoders[i]);
        std::cout << "M" << i+1 << ":" << std::setw(7) << encoders[i] << " (" << std::fixed << std::setprecision(1) << deg << "°)  " << std::endl;
      }
      std::cout << "\r" << std::flush;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

  } catch (const Exception & e) {
    std::cerr << "\n[KNI ERROR] " << e.message() << "\n";
    return 1;
  } catch (const std::exception & e) {
    std::cerr << "\n[ERROR] " << e.what() << "\n";
    return 1;
  }

  std::cout << "\nGoodbye.\n";
  return 0;
}
