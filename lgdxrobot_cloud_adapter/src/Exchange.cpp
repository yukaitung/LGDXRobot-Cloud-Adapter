#include "lgdxrobot_cloud_adapter/Exchange.hpp"

//Cloud Exchange
CloudExchange::CloudExchange(RobotClientsService::Stub *stub, std::shared_ptr<grpc::CallCredentials> accessToken, std::shared_ptr<CloudSignals> cloudSignalsPtr)
{
  cloudExchangeStream = std::make_unique<CloudExchangeStream>(stub, accessToken, cloudSignalsPtr);
}

void CloudExchange::SendMessage(const RobotClientsData &robotData, const RobotClientsNextToken &nextToken, const RobotClientsAbortToken &abortToken)
{
  cloudExchangeStream->SendMessage(robotData, nextToken, abortToken);
}

void CloudExchange::Shutdown()
{
  cloudExchangeStream->Shutdown();
}

grpc::Status CloudExchange::AwaitCompletion()
{
  return cloudExchangeStream->AwaitCompletion();
}

//Slam Exchange
SlamExchange::SlamExchange(RobotClientsService::Stub *stub, std::shared_ptr<grpc::CallCredentials> accessToken, std::shared_ptr<CloudSignals> cloudSignalsPtr)
{
  slamExchangeStream = std::make_unique<SlamExchangeStream>(stub, accessToken, cloudSignalsPtr);
}

void SlamExchange::SendMessage(const RobotClientsSlamStatus status, const RobotClientsData &robotData, const RobotClientsMapData &mapData)
{
  slamExchangeStream->SendMessage(status, robotData, mapData);
}

void SlamExchange::Shutdown()
{
  slamExchangeStream->Shutdown();
}

grpc::Status SlamExchange::AwaitCompletion()
{
  return slamExchangeStream->AwaitCompletion();
}