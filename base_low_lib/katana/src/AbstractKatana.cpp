/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2010  University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * AbstractKatana.cpp
 *
 *  Created on: 20.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#include <katana/AbstractKatana.h>

namespace katana
{

AbstractKatana::AbstractKatana(rclcpp::Node::SharedPtr node) : node_(node)
{
  // names and types for the 5 "real" joints
  joint_names_.resize(NUM_JOINTS);
  joint_types_.resize(NUM_JOINTS);

  // names and types for the 2 "finger" joints

  gripper_joint_names_.resize(NUM_GRIPPER_JOINTS);
  gripper_joint_types_.resize(NUM_GRIPPER_JOINTS);

  // angles and velocities and limits: the 5 "real" joints + gripper
  motor_angles_.resize(NUM_MOTORS);
  motor_velocities_.resize(NUM_MOTORS);
  motor_limits_.resize(NUM_MOTORS + 1);

  /* ********* get parameters ********* */
  std::string robot_desc_string;

  if (!node->get_parameter("robot_description", robot_desc_string))
  {
    RCLCPP_FATAL(node->get_logger(), "Couldn't get a robot_description from the param server");
    return;
  }

  urdf::Model model;
  model.initString(robot_desc_string);

  std::vector<std::string> joint_names;
  // Gets all of the joints
  node->declare_parameter("katana_joints", std::vector<std::string>());
  if (!node->get_parameter("katana_joints", joint_names) || joint_names.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "No joints given.");
  }
  if (joint_names.size() != (size_t)NUM_JOINTS)
  {
    RCLCPP_ERROR(node->get_logger(), "Wrong number of joints! was: %zu, expected: %zu", joint_names.size(), NUM_JOINTS);
  }
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    joint_names_[i] = joint_names[i];
    joint_types_[i] = urdf::Joint::REVOLUTE; // all of our joints are of type revolute

    motor_limits_[i].joint_name = joint_names_[i];
    motor_limits_[i].min_position = model.getJoint(joint_names_[i])->limits->lower;
    motor_limits_[i].max_position = model.getJoint(joint_names_[i])->limits->upper;
  }

  std::vector<std::string> gripper_joint_names;
  node->declare_parameter("katana_gripper_joints", std::vector<std::string>());
  // Gets all of the joints
  if (!node->get_parameter("katana_gripper_joints", gripper_joint_names) || gripper_joint_names.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "No gripper_joints given.");
  }
  if ((size_t)gripper_joint_names.size() != NUM_GRIPPER_JOINTS)
  {
    RCLCPP_ERROR(node->get_logger(), "Wrong number of gripper_joints! was: %zu, expected: %zu", gripper_joint_names.size(), NUM_GRIPPER_JOINTS);
  }
  for (size_t i = 0; i < NUM_GRIPPER_JOINTS; ++i)
  {
    gripper_joint_names_[i] = gripper_joint_names[i];
    gripper_joint_types_[i] = urdf::Joint::REVOLUTE; // all of our joints are of type revolute

    motor_limits_[NUM_JOINTS + i].joint_name = gripper_joint_names_[i];
    motor_limits_[NUM_JOINTS + i].min_position = model.getJoint(gripper_joint_names_[i])->limits->lower;
    motor_limits_[NUM_JOINTS + i].max_position = model.getJoint(gripper_joint_names_[i])->limits->upper;
  }
}

AbstractKatana::~AbstractKatana()
{
}

void AbstractKatana::freezeRobot()
{
  // do nothing (can be overridden)
}

void AbstractKatana::refreshMotorStatus()
{
  // do nothing (can be overridden)
}

/* ******************************** joints + motors ******************************** */

int AbstractKatana::getJointIndex(std::string joint_name)
{
  for (int i = 0; i < (int)joint_names_.size(); i++)
  {
    if (joint_names_[i] == joint_name)
      return i;
  }

  for (int i = 0; i < (int)gripper_joint_names_.size(); i++)
  {
    if (gripper_joint_names_[i] == joint_name)
      return GRIPPER_INDEX;
  }

  RCLCPP_ERROR(rclcpp::get_logger("katana"), "Joint not found: %s.", joint_name.c_str());
  return -1;
}

std::vector<std::string> AbstractKatana::getJointNames()
{
  return joint_names_;
}

std::vector<int> AbstractKatana::getJointTypes()
{
  return joint_types_;
}

std::vector<std::string> AbstractKatana::getGripperJointNames()
{
  return gripper_joint_names_;
}

std::vector<int> AbstractKatana::getGripperJointTypes()
{
  return gripper_joint_types_;
}

std::vector<double> AbstractKatana::getMotorAngles()
{
  return motor_angles_;
}

std::vector<double> AbstractKatana::getMotorVelocities()
{
  return motor_velocities_;
}

std::vector<JointLimit> AbstractKatana::getMotorLimits()
{
  return motor_limits_;
}

double AbstractKatana::getMotorLimitMax(std::string joint_name)
{
  for (size_t i = 0; i < motor_limits_.size(); i++)
  {
    if (motor_limits_[i].joint_name == joint_name)
    {
      return motor_limits_[i].max_position;
    }
  }

  return -1;
}

double AbstractKatana::getMotorLimitMin(std::string joint_name)
{
  for (size_t i = 0; i < motor_limits_.size(); i++)
  {
    if (motor_limits_[i].joint_name == joint_name)
    {
      return motor_limits_[i].min_position;
    }
  }

  return -1;
}

}
