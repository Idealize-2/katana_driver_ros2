/*
 * JointStatePublisher.h
 */

#ifndef JOINTSTATEPUBLISHER_H_
#define JOINTSTATEPUBLISHER_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <vector>
#include <memory>

#include <katana/AbstractKatana.h>

namespace katana
{

class JointStatePublisher
{
public:
  JointStatePublisher(std::shared_ptr<AbstractKatana> katana, rclcpp::Node::SharedPtr node);
  virtual ~JointStatePublisher();
  void update();

private:
  std::shared_ptr<AbstractKatana> katana_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;

};

}

#endif /* JOINTSTATEPUBLISHER_H_ */
