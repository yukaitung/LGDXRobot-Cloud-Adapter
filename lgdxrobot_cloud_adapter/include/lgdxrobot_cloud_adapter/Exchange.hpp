#ifndef EXCHANGE_HPP
#define EXCHANGE_HPP

#include "proto/RobotClientsService.grpc.pb.h"
#include "CloudExchangeStream.hpp"
#include "SlamExchangeStream.hpp"

class IExchange
{
  public:
    virtual void SendMessage([[maybe_unused]] const RobotClientsData &robotData,
      [[maybe_unused]] const RobotClientsNextToken &nextToken,
      [[maybe_unused]] const RobotClientsAbortToken &abortToken) {};
    virtual void SendMessage([[maybe_unused]] const RobotClientsSlamStatus status,
      [[maybe_unused]] const RobotClientsData &robotData,
      [[maybe_unused]] const RobotClientsMapData &mapData) {};  
    virtual void Shutdown() = 0;
    virtual grpc::Status AwaitCompletion() = 0;
};

class CloudExchangeType : public IExchange
{
    private:
    std::unique_ptr<CloudExchangeStream> cloudExchangeStream;

  public:
    CloudExchangeType(RobotClientsService::Stub *stub, std::shared_ptr<grpc::CallCredentials> accessToken, std::shared_ptr<CloudSignals> cloudSignalsPtr);
    void SendMessage(const RobotClientsData &robotData, const RobotClientsNextToken &nextToken, const RobotClientsAbortToken &abortToken) override;
    void Shutdown() override;
    grpc::Status AwaitCompletion() override;
};

class SlamExchangeType : public IExchange
{
    private:
    std::unique_ptr<SlamExchangeStream> slamExchangeStream;

  public:
    SlamExchangeType(RobotClientsService::Stub *stub, std::shared_ptr<grpc::CallCredentials> accessToken, std::shared_ptr<CloudSignals> cloudSignalsPtr);
    void SendMessage(const RobotClientsSlamStatus status, const RobotClientsData &robotData, const RobotClientsMapData &mapData) override;
    void Shutdown() override;
    grpc::Status AwaitCompletion() override;
};

#endif // EXCHANGE_HPP