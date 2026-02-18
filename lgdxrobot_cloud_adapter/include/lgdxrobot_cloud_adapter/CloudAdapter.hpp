#ifndef CLOUD_ADAPTER_HPP
#define CLOUD_ADAPTER_HPP

#include <string>

#include "grpc/grpc.h"
#include "grpcpp/channel.h"
#include "grpcpp/client_context.h"
#include "grpcpp/create_channel.h"
#include "grpcpp/security/credentials.h"
#include "proto/RobotClientsService.grpc.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "RobotStatus.hpp"

namespace LGDXRobotCloud
{

struct CloudErrorRetryData
{
  std::string mcuSerialNumber;
  RobotClientsNextToken nextToken;
  RobotClientsAbortToken abortToken;
};

class CloudAdapter : public rclcpp::Node
{
  private:
  const int kGrpcWaitSec = 5;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr cloudRetryTimer;
    
    std::shared_ptr<grpc::Channel> grpcChannel;
    std::unique_ptr<RobotClientsService::Stub> grpcRealtimeStub;
    std::unique_ptr<RobotClientsService::Stub> grpcStub;
    std::shared_ptr<grpc::CallCredentials> accessToken;

    RobotStatus::StateMachine robotStatus = RobotStatus::Offline();
    CloudErrorRetryData cloudErrorRetryData;

    std::string ReadCertificate(const char *filename);
    #ifdef __linux__ 
    std::string GetMotherBoardSerialNumber();
    #endif
    void SetSystemInfo(RobotClientsSystemInfo *info);
    void HandleError();

  public:
    CloudAdapter(const rclcpp::NodeOptions &options);
    void Initalise();

    void Greet(std::string mcuSN);
    void Exchange(const RobotClientsData &robotData,
      const RobotClientsNextToken &nextToken,
      const RobotClientsAbortToken &abortToken);
    void SlamExchange(const RobotClientsSlamStatus status,
      const RobotClientsData &robotData,
      const RobotClientsMapData &mapData);
    void OnErrorOccured();
};

}

#endif // CLOUD_ADAPTER_HPP