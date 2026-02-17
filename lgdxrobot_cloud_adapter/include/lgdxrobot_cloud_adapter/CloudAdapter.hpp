#ifndef CLOUD_ADAPTER_HPP
#define CLOUD_ADAPTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "RobotStatus.hpp"

namespace LGDXRobotCloud
{
  
class CloudAdapter : public rclcpp::Node
{
  private:
    rclcpp::TimerBase::SharedPtr timer;
    RobotStatus::StateMachine robotStatus = RobotStatus::Offline();

  public:
    CloudAdapter(const rclcpp::NodeOptions &options);

    void Initalise();
};

}

#endif // CLOUD_ADAPTER_HPP