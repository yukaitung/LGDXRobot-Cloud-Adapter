#include "lgdxrobot_cloud_adapter/CloudAdapter.hpp"
#include "lgdxrobot_cloud_adapter/RobotStatus.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CloudAdapter>();
  node->Initalise();
  rclcpp::spin(node);
  return 0;
}