// =============================================================================
// Tutorial 3: offset_inverse_kinematics.cpp
// =============================================================================
// ROS 2 port — Full 3D IK with DH-parameter link offsets
//
// WHAT IT DOES:
//   Most complete IK version. Handles the real geometry of the Katana arm
//   including a link offset (d) between joint 1 and the shoulder joint.
//
// IK MODEL (DH-parameter based):
//   a2 = 35 cm, a3 = 25 cm, d1 = 0 (no offset for this arm variant)
//
//   θ1 = atan2(y,x) + atan2(-sqrt(x²+y²-d²), d)
//   θ3 = atan2(-sqrt(1-D²), D)       where D = (x²+y²+z²-d²-a2²-a3²)/(2·a2·a3)
//   θ2 = atan2(z, sqrt(x²+y²-d²)) - atan2(a3·sin(θ3), a2+a3·cos(θ3))
//
// RUN:
//   ros2 run katana_tutorials offset_inverse_kinematics
//   Then type: X Y Z  (e.g.  20 10 15)
// =============================================================================

#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "katana_tutorials/katana_arm_client.hpp"

// ---------------------------------------------------------------------------
// IK math (same as the original offset_inverse_kinematics.cpp)
// ---------------------------------------------------------------------------

/// Elbow angle using proper atan2 formulation (correct quadrant)
double Theta_3(double a2, double a3, double x, double y, double z, double d)
{
  double D = (x*x + y*y + z*z - d*d - a2*a2 - a3*a3) / (2.0 * a2 * a3);
  D = std::max(-1.0, std::min(1.0, D));
  return std::atan2(-std::sqrt(std::max(0.0, 1.0 - D*D)), D);
}

/// Shoulder angle with offset compensation
double Theta_2(double a2, double a3, double x, double y, double z,
               double d, double theta3)
{
  double s3 = std::sin(theta3);
  double c3 = std::cos(theta3);
  double xy_plane = std::sqrt(std::max(0.0, x*x + y*y - d*d));
  return std::atan2(z, xy_plane) - std::atan2(a3 * s3, a2 + a3 * c3);
}

/// Pan angle with link offset
double Theta_1(double x, double y, double d)
{
  if (std::abs(d) < 1e-9) {
    return std::atan2(y, x);
  }
  double xy = std::sqrt(std::max(0.0, x*x + y*y - d*d));
  return std::atan2(y, x) + std::atan2(-xy, d);
}

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto arm = std::make_shared<katana_tutorials::KatanaArmClient>(
    "offset_inverse_kinematics");

  if (!arm->waitForJointState(10.0)) {
    RCLCPP_FATAL(arm->get_logger(), "Could not get joint state.");
    rclcpp::shutdown();
    return 1;
  }

  const double a2 = 35.0;
  const double a3 = 25.0;
  const double d  = 0.0;  // link 1 offset — 0 for Katana 6M 180 variant
  const double reach = a2 + a3;

  // ── Read target from user ───────────────────────────────────────────────
  double x, y, z;
  std::cout << "\nKatana Full 3D IK (with link offsets)\n";
  std::cout << "Arm reach: " << reach << " cm\n";
  std::cout << "Enter target X Y Z (in cm, e.g. 20 10 15): ";
  std::cin >> x >> y >> z;

  // ── Reachability check ───────────────────────────────────────────────────
  double p  = std::sqrt(x*x + y*y);
  double r3 = std::sqrt(p*p + z*z);
  if (p > reach || std::abs(z) > reach || r3 > reach) {
    RCLCPP_WARN(arm->get_logger(),
      "Target out of reach (distance=%.1f cm, max=%.1f cm). Using safe default.",
      r3, reach);
    x = 60.0; y = 0.0; z = 0.0;
  }

  // ── Solve IK ─────────────────────────────────────────────────────────────
  double theta3 = Theta_3(a2, a3, x, y, z, d);
  double theta2 = Theta_2(a2, a3, x, y, z, d, theta3);
  double theta1 = (std::abs(x) > 1e-6 || std::abs(y) > 1e-6)
                  ? Theta_1(x, y, d)
                  : arm->currentJointPositions()[0];

  std::vector<double> target_pos = {
    theta1,   // motor1 pan
    theta2,   // motor2 shoulder lift
    0.0,      // motor3 — not driven in this model
    -theta3,  // motor4 elbow (sign matches Katana convention)
    0.0       // motor5 wrist roll
  };

  RCLCPP_INFO(arm->get_logger(),
    "IK (offset) solution: θ1=%.3f  θ2=%.3f  θ3=%.3f", theta1, theta2, theta3);
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
