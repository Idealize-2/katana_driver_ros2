// =============================================================================
// Tutorial 6: test_inverse_kinematics.cpp
// =============================================================================
// ROS 2 — 2D Inverse Kinematics (planar, pan = 0) for the Katana 400 6M180.
//
// WHAT IT DOES:
//   User types an X (forward), Y (up) target in the sagittal plane (pan = 0).
//   The program solves the 2-link planar IK (law of cosines) and moves the arm.
//
// IK MODEL:
//   Link lengths from katana6M180.cfg [ENDEFFECTOR] (in cm):
//     a2 = 19.10 cm  (segment2 = 191.0 mm — shoulder to elbow)
//     a3 = 31.33 cm  (segment3 + segment4 = 313.3 mm — elbow to TCP)
//   Maximum reach ≈ 50.4 cm.
//
//   alpha = acos((a2²-a3²+x²+y²) / (2·a2·sqrt(x²+y²)))   shoulder angle
//   beta  = acos((a2²+a3²-x²-y²) / (2·a2·a3))             elbow angle
//   elev  = atan2(y, x)                                     elevation of target
//
//   motor1 (pan)      = 0   — arm always faces forward in this 2D model
//   motor2 (shoulder) = elev + k·alpha
//   motor3            = 0
//   motor4 (elbow)    = k·(π - beta)
//   motor5            = 0
//
//   For x < 0 (target behind): k = -1 (arm bends backward)
//
// REQUIRES:
//   ros2_control driver running (real_hardware.launch.py or full_system.launch.py)
//
// RUN:
//   ros2 run katana_tutorials test_inverse_kinematics
//   Then type: X Y  (in cm, e.g.  20 10)
// =============================================================================

#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "katana_tutorials/katana_arm_client.hpp"

// ---------------------------------------------------------------------------
// 2-link planar IK helpers
// ---------------------------------------------------------------------------

/// Shoulder angle (alpha) — law of cosines, upper triangle
static double alpha_cal(double a2, double a3, double x, double y)
{
  double denom = 2.0 * a2 * std::sqrt(x * x + y * y);
  if (std::abs(denom) < 1e-9) { return 0.0; }
  double cos_val = (a2*a2 - a3*a3 + x*x + y*y) / denom;
  cos_val = std::max(-1.0, std::min(1.0, cos_val));
  return std::acos(cos_val);
}

/// Elbow angle (beta) — law of cosines, full triangle
static double beta_cal(double a2, double a3, double x, double y)
{
  double denom = 2.0 * a2 * a3;
  if (std::abs(denom) < 1e-9) { return 0.0; }
  double cos_val = (a2*a2 + a3*a3 - x*x - y*y) / denom;
  cos_val = std::max(-1.0, std::min(1.0, cos_val));
  return std::acos(cos_val);
}

/// Elevation angle of the target point above horizontal
static double elevAngle(double x, double y)
{
  return std::atan2(y, x);
}

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto arm = std::make_shared<katana_tutorials::KatanaArmClient>(
    "test_inverse_kinematics");

  RCLCPP_INFO(arm->get_logger(), "Waiting for joint state...");
  if (!arm->waitForJointState(15.0)) {
    RCLCPP_FATAL(arm->get_logger(), "No joint state received — is the driver running?");
    rclcpp::shutdown();
    return 1;
  }

  // Katana 400 6M180 link lengths (cm)
  const double a2    = 19.10;   // segment2 = 191.0 mm
  const double a3    = 31.33;   // segment3 + segment4 = 313.3 mm
  const double reach = a2 + a3; // ≈ 50.4 cm

  // ── Read target from user ─────────────────────────────────────────────────
  double x, y;
  std::cout << "\nKatana 400 6M180 — 2D Planar IK (pan = 0, arm faces forward)\n";
  std::cout << "Arm reach: " << reach << " cm  (a2=" << a2 << ", a3=" << a3 << ")\n";
  std::cout << "Enter target X Y (in cm, X=forward, Y=up, e.g.  20 10): ";
  if (!(std::cin >> x >> y)) {
    std::cerr << "  [!] Invalid input.\n";
    rclcpp::shutdown();
    return 1;
  }

  // ── Reachability check ────────────────────────────────────────────────────
  double dist = std::sqrt(x * x + y * y);
  if (dist > reach) {
    RCLCPP_WARN(arm->get_logger(),
      "Target (%.1f, %.1f) cm is %.2f cm away — beyond reach of %.2f cm. "
      "Using safe default (20, 10).", x, y, dist, reach);
    x = 20.0;
    y = 10.0;
    dist = std::sqrt(x * x + y * y);
  }

  // ── Solve IK ──────────────────────────────────────────────────────────────
  // For x < 0 the arm bends backward: k = -1
  const double k    = (x < 0.0) ? -1.0 : 1.0;
  const double elev  = elevAngle(x, y);
  const double alpha = alpha_cal(a2, a3, x, y);
  const double beta  = beta_cal(a2, a3, x, y);

  std::vector<double> target_pos = {
    0.0,                         // motor1 (pan)     — fixed at 0 for 2D model
    elev + k * alpha,            // motor2 (shoulder)
    0.0,                         // motor3            — not used
    k * (M_PI - beta),           // motor4 (elbow)
    0.0                          // motor5 (wrist roll) — not used
  };

  RCLCPP_INFO(arm->get_logger(),
    "IK solution: elev=%.3f  alpha=%.3f  beta=%.3f  k=%.0f",
    elev, alpha, beta, k);
  RCLCPP_INFO(arm->get_logger(),
    "Motor targets: [%.3f, %.3f, %.3f, %.3f, %.3f]",
    target_pos[0], target_pos[1], target_pos[2], target_pos[3], target_pos[4]);

  // ── Build and send trajectory ─────────────────────────────────────────────
  auto current = arm->currentJointPositions();
  auto traj    = katana_tutorials::makeTrajectory(
    katana_tutorials::KatanaArmClient::ARM_JOINT_NAMES,
    current, target_pos, 5.0, 6.0);

  if (!arm->sendTrajectoryAndWait(traj)) {
    RCLCPP_ERROR(arm->get_logger(), "Movement failed.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(arm->get_logger(), "Reached target position!");
  rclcpp::shutdown();
  return 0;
}
