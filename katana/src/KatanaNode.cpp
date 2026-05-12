/*
 * KatanaNode.cpp
 */

#include <katana/KatanaNode.h>

namespace katana
{

KatanaNode::KatanaNode(const rclcpp::NodeOptions& options)
: Node("katana", options)
{
}

void KatanaNode::init()
{
  bool simulation = this->declare_parameter("simulation", false);
  std::string katana_type = this->declare_parameter("katana_type", "");

  if (simulation)
  {
    katana_ = std::make_shared<SimulatedKatana>(shared_from_this());
  }
  else
  {
    if (katana_type.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter katana_type was not set!");
      rclcpp::shutdown();
      return;
    }

    if (katana_type == "katana_300_6m180")
      katana_ = std::make_shared<Katana300>(shared_from_this());
    else if (katana_type == "katana_400_6m180" || katana_type == "katana_450_6m90a"
        || katana_type == "katana_450_6m90b")
      katana_ = std::make_shared<Katana>(shared_from_this());
    else
    {
      RCLCPP_ERROR(this->get_logger(),
          "Parameter katana_type was set to invalid value: %s; please use one of the following: katana_300_6m180, katana_400_6m180, katana_450_6m90a, katana_450_6m90b", katana_type.c_str());
      rclcpp::shutdown();
      return;
    }
  }

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(40), // 25 Hz
    std::bind(&KatanaNode::loop, this));
}

KatanaNode::~KatanaNode()
{
}

void KatanaNode::loop()
{
  katana_->refreshEncoders();
}

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<katana::KatanaNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
