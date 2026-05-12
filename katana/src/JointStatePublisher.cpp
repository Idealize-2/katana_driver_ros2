/*
 * JointStatePublisher.cpp
 */

#include "katana/JointStatePublisher.h"

namespace katana
{

JointStatePublisher::JointStatePublisher(std::shared_ptr<AbstractKatana> katana, rclcpp::Node::SharedPtr node) :
  katana_(katana),
  node_(node)
{
  pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
}

JointStatePublisher::~JointStatePublisher()
{
}

void JointStatePublisher::update()
{
  /* ************** Publish joint angles ************** */
  auto msg = std::make_unique<sensor_msgs::msg::JointState>();
  std::vector<std::string> joint_names = katana_->getJointNames();
  std::vector<double> angles = katana_->getMotorAngles();
  std::vector<double> vels = katana_->getMotorVelocities();

  for (size_t i = 0; i < NUM_JOINTS; i++)
  {
    msg->name.push_back(joint_names[i]);
    msg->position.push_back(angles[i]);
    msg->velocity.push_back(vels[i]);
  }

  msg->name.push_back(katana_->getGripperJointNames()[0]);
  msg->position.push_back(angles[5]);
  msg->velocity.push_back(vels[5]);

  msg->name.push_back(katana_->getGripperJointNames()[1]);
  msg->position.push_back(angles[5]); // both right and left finger are controlled by motor 6
  msg->velocity.push_back(vels[5]);

  msg->header.stamp = node_->now();
  pub_->publish(std::move(msg));
}

}
