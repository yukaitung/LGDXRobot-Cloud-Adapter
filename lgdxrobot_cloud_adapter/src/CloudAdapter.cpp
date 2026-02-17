#include <chrono>
#include <rclcpp_components/register_node_macro.hpp>
#include "lgdxrobot_cloud_adapter/CloudAdapter.hpp"

namespace LGDXRobotCloud
{
  
CloudAdapter::CloudAdapter(const rclcpp::NodeOptions &options) : Node("lgdxrobot_cloud_adapter_node", options)
{
  timer = this->create_wall_timer(std::chrono::microseconds(1), [this]() {this->Initalise();});
}

void CloudAdapter::Initalise()
{
  timer->cancel();

  RCLCPP_INFO(this->get_logger(), "Initialising Cloud Adapter");
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(LGDXRobotCloud::CloudAdapter)