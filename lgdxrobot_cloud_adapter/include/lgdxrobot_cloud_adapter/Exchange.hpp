#ifndef EXCHANGE_HPP
#define EXCHANGE_HPP

#include "proto/RobotClientsService.grpc.pb.h"
#include "CloudExchangeStream.hpp"
#include "SlamExchangeStream.hpp"

class IExchange
{
  public:
    virtual void SendMessage(const RobotClientsData &robotData,
      const RobotClientsNextToken &nextToken,
      const RobotClientsAbortToken &abortToken) = 0;
    virtual void SendMessage(const RobotClientsSlamStatus status,
      const RobotClientsData &robotData,
      const RobotClientsMapData &mapData) = 0;  
    virtual void Shutdown() = 0;
    virtual grpc::Status AwaitCompletion() = 0;
};

class CloudExchange : public IExchange
{
    private:
    std::unique_ptr<CloudExchangeStream> cloudExchangeStream;

  public:
    CloudExchange(RobotClientsService::Stub *stub, std::shared_ptr<grpc::CallCredentials> accessToken, std::shared_ptr<CloudSignals> cloudSignalsPtr);
    void SendMessage(const RobotClientsData &robotData, const RobotClientsNextToken &nextToken, const RobotClientsAbortToken &abortToken) override;
    void SendMessage(const RobotClientsSlamStatus, const RobotClientsData, const RobotClientsMapData) {}
    void Shutdown() override;
    grpc::Status AwaitCompletion() override;
};

class SlamExchange : public IExchange
{
    private:
    std::unique_ptr<SlamExchangeStream> slamExchangeStream;

  public:
    SlamExchange(RobotClientsService::Stub *stub, std::shared_ptr<grpc::CallCredentials> accessToken, std::shared_ptr<CloudSignals> cloudSignalsPtr);
    void SendMessage(const RobotClientsData, const RobotClientsNextToken, const RobotClientsAbortToken) {}
    void SendMessage(const RobotClientsSlamStatus status, const RobotClientsData &robotData, const RobotClientsMapData &mapData) override;
    void Shutdown() override;
    grpc::Status AwaitCompletion() override;
};

#endif // EXCHANGE_HPP