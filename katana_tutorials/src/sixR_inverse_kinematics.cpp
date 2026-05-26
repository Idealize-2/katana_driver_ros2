// =============================================================================
// Tutorial 2: sixR_inverse_kinematics.cpp
// =============================================================================
// ROS 2 — 3D Inverse Kinematics for the Katana 400 6M180 arm (no link offsets)
//
// WHAT IT DOES:
//   User types an X, Y, Z target (in cm, relative to the base).
//   The program calculates the joint angles (θ1, θ2, θ3) using geometric IK
//   and moves the arm to that position.
//
// IK MODEL:
//   The Katana 400 6M180 is modelled as two links in a vertical plane.
//   Link lengths are taken from katana6M180.cfg [ENDEFFECTOR]:
//     a2 = 19.1 cm  (segment2 = 191.0 mm — shoulder to elbow)
//     a3 = 31.33 cm (segment3 + segment4 = 147.3 + 166.0 = 313.3 mm — elbow to TCP)
//   Joint 1 (pan)  = atan2(y, x)
//   Joint 2 (lift) = law-of-cosines shoulder angle
//   Joint 3 (lift) = law-of-cosines elbow angle
//   Joints 4, 5 = fixed at 0 (this is the simplified 3R model)
//
// NOTE: For exact reachability validation use Tutorial 4 (ik_pose_mover) which
//       calls KNI's own IKCalculate() with the full kinematic model.
//
// RUN:
//   ros2 run katana_tutorials sixR_inverse_kinematics
//   Then type: X Y Z  (in cm, e.g.  15 10 10)
// =============================================================================

#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "katana_tutorials/katana_arm_client.hpp"

// ---------------------------------------------------------------------------
// IK math (same as the original sixR_inverse_kinematics.cpp)
// ---------------------------------------------------------------------------

/// Elbow angle (joint 3 equivalent) using law of cosines
double Theta_3(double a2, double a3, double d1, double z)
{
  double cos_val = (a2*a2 + a3*a3 - d1*d1 - z*z) / (2.0 * a2 * a3);
  // Clamp to [-1, 1] to avoid acos domain errors from floating point noise
  cos_val = std::max(-1.0, std::min(1.0, cos_val));
  return std::acos(cos_val);
}

/// Shoulder angle (joint 2 equivalent)
double Theta_2(double a2, double a3, double d1, double z)
{
  double denom = 2.0 * a2 * std::sqrt(d1*d1 + z*z);
  if (std::abs(denom) < 1e-9) return 0.0;
  double cos_val = (a2*a2 - a3*a3 + d1*d1 + z*z) / denom;
  cos_val = std::max(-1.0, std::min(1.0, cos_val));
  return std::acos(cos_val);
}

/// Pan angle (joint 1)
double Theta_1(double x, double y)
{
  return std::atan2(y, x);
}

/// Wrist angle correction — angle of the tool tip in the elevation plane
double finalPointAngle(double d1, double z)
{
  return std::atan2(z, d1);
}

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto arm = std::make_shared<katana_tutorials::KatanaArmClient>(
    "sixR_inverse_kinematics");

  if (!arm->waitForJointState(10.0)) {
    RCLCPP_FATAL(arm->get_logger(), "Could not get joint state.");
    rclcpp::shutdown();
    return 1;
  }

  // Katana 400 6M180 segment lengths (from katana6M180.cfg [ENDEFFECTOR], in cm)
  //   segment2 = 191.0 mm              →  19.10 cm  (shoulder to elbow)
  //   segment3 + segment4 = 313.3 mm   →  31.33 cm  (elbow to TCP)
  const double a2 = 19.10;
  const double a3 = 31.33;
  const double reach = a2 + a3;   // maximum reach ≈ 50.4 cm

  // ── Read target from user ───────────────────────────────────────────────
  double x, y, z;
  std::cout << "\nKatana 400 6M180 — 3D IK (simplified 3R model)\n";
  std::cout << "Arm reach: " << reach << " cm  (a2=" << a2 << " cm, a3=" << a3 << " cm)\n";
  std::cout << "Enter target X Y Z (in cm, e.g. 15 10 10): ";
  std::cin >> x >> y >> z;

  // ── Reachability check ───────────────────────────────────────────────────
  double d1 = std::sqrt(x*x + y*y);   // horizontal distance from base
  double r3d = std::sqrt(d1*d1 + z*z);// full 3D distance

  if (r3d > reach) {
    RCLCPP_WARN(arm->get_logger(),
      "Target (%.1f, %.1f, %.1f) is %.1f cm away — beyond reach of %.1f cm. "
      "Clamping to a safe default.", x, y, z, r3d, reach);
    x = 20.0; y = 30.0; z = 0.0;
    d1 = std::sqrt(x*x + y*y);
  }

  // ── Solve IK ─────────────────────────────────────────────────────────────
  double theta3 = Theta_3(a2, a3, d1, z);
  double theta2 = Theta_2(a2, a3, d1, z);
  double theta1 = (std::abs(x) > 1e-6 || std::abs(y) > 1e-6)
                  ? Theta_1(x, y)
                  : arm->currentJointPositions()[0];  // keep current pan if x=y=0

  double elev   = finalPointAngle(d1, z);

  // Map IK angles to Katana motor commands
  // motor1 = theta1 (pan)
  // motor2 = elev + theta2 (shoulder lift, corrected for elevation)
  // motor3 = 0  (middle lift — not used in this simplified model)
  // motor4 = PI - theta3 (elbow, sign convention from original code)
  // motor5 = 0  (wrist roll — not used here)
  std::vector<double> target_pos = {
    theta1,
    elev + theta2,
    0.0,
    3.14159 - theta3,
    0.0
  };

  RCLCPP_INFO(arm->get_logger(),
    "IK solution: θ1=%.3f  θ2=%.3f  θ3=%.3f", theta1, theta2, theta3);
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
