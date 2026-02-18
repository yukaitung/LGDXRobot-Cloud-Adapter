#include "lgdxrobot_cloud_adapter/Exchange.hpp"

//CloudExchangeType
CloudExchangeType::CloudExchangeType(RobotClientsService::Stub *stub, std::shared_ptr<grpc::CallCredentials> accessToken, std::shared_ptr<CloudSignals> cloudSignalsPtr)
{
  cloudExchangeStream = std::make_unique<CloudExchangeStream>(stub, accessToken, cloudSignalsPtr);
}

void CloudExchangeType::SendMessage(const RobotClientsData &robotData, const RobotClientsNextToken &nextToken, const RobotClientsAbortToken &abortToken)
{
  cloudExchangeStream->SendMessage(robotData, nextToken, abortToken);
}

void CloudExchangeType::Shutdown()
{
  cloudExchangeStream->Shutdown();
}

grpc::Status CloudExchangeType::AwaitCompletion()
{
  return cloudExchangeStream->AwaitCompletion();
}

//Slam Exchange
SlamExchangeType::SlamExchangeType(RobotClientsService::Stub *stub, std::shared_ptr<grpc::CallCredentials> accessToken, std::shared_ptr<CloudSignals> cloudSignalsPtr)
{
  slamExchangeStream = std::make_unique<SlamExchangeStream>(stub, accessToken, cloudSignalsPtr);
}

void SlamExchangeType::SendMessage(const RobotClientsSlamStatus status, const RobotClientsData &robotData, const RobotClientsMapData &mapData)
{
  slamExchangeStream->SendMessage(status, robotData, mapData);
}

void SlamExchangeType::Shutdown()
{
  slamExchangeStream->Shutdown();
}

grpc::Status SlamExchangeType::AwaitCompletion()
{
  return slamExchangeStream->AwaitCompletion();
}