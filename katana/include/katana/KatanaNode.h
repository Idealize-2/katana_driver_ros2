/*
 * KatanaNode.h
 */

#ifndef KATANANODE_H_
#define KATANANODE_H_

#include <rclcpp/rclcpp.hpp>

#include <katana/AbstractKatana.h>
#include <katana/Katana.h>
#include <katana/Katana300.h>
#include <katana/SimulatedKatana.h>

#include <memory>

namespace katana
{

class KatanaNode : public rclcpp::Node
{
public:
  KatanaNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  virtual ~KatanaNode();

  void init();

private:
  void loop();

  std::shared_ptr<katana::AbstractKatana> katana_;
  
  rclcpp::TimerBase::SharedPtr timer_;
};

}

#endif /* KATANANODE_H_ */
