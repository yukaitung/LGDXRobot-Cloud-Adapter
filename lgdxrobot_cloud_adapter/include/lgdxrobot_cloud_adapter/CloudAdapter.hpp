#ifndef CLOUD_ADAPTER_HPP
#define CLOUD_ADAPTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "RobotStatus.hpp"

class CloudAdapter : public rclcpp::Node
{
  private:
    RobotStatus::StateMachine robotStatus = RobotStatus::Offline();

  public:
    CloudAdapter();

    void Initalise();
};

#endif // CLOUD_ADAPTER_HPP