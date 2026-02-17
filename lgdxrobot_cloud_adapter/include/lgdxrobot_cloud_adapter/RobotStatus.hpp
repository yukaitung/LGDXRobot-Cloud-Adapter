#ifndef ROBOT_STATUS_HPP
#define ROBOT_STATUS_HPP

#include <variant>

#include "proto/RobotClientsService.pb.h"

namespace RobotStatus
{

class Idle;
class Running;
class Stuck;
class Aborting;
class Paused;
class Critical;
class Charging;
class Offline;

using StateMachine = std::variant<std::monostate, Idle, Running, Stuck, Aborting, Paused, Critical, Charging, Offline>;

class IBase 
{
  public:
    virtual ~IBase() = default;
    virtual RobotClientsRobotStatus GetStatus() const {return RobotClientsRobotStatus::Offline;}
};

class Idle : public IBase
{
  friend class Offline;
  friend class Charging;
  friend class Running;
  friend class Stuck;
  friend class Paused;
  friend class Aborting;
  Idle() = default;
  public:
    RobotClientsRobotStatus GetStatus() const override {return RobotClientsRobotStatus::Idle;}
    Charging StartCharging();
    Running TaskAssigned();
    Paused PauseTaskAssignment();
};

class Running : public IBase
{
  friend class Idle;
  friend class Aborting;
  friend class Stuck;
  Running() = default;
  public:
    RobotClientsRobotStatus GetStatus() const override {return RobotClientsRobotStatus::Running;}
    Idle TaskCompleted();
    Stuck NaviagtionStuck();
    Aborting AbortTask();
};

class Stuck : public IBase
{
  friend class Running;
  Stuck() = default;
  public:
    RobotClientsRobotStatus GetStatus() const override {return RobotClientsRobotStatus::Stuck;}
    Idle TaskCompleted();
    Running Cleared();
    Aborting AbortTask();
};

class Aborting : public IBase
{
  friend class Running;
  friend class Stuck;
  Aborting() = default;
  public:
    RobotClientsRobotStatus GetStatus() const override {return RobotClientsRobotStatus::Aborting;}
    Running TaskAssigned();
    Idle TaskAborted();
};

class Paused : public IBase
{
  friend class Idle;
  Paused() = default;
  public:
    RobotClientsRobotStatus GetStatus() const override {return RobotClientsRobotStatus::Paused;}
    Idle ResumeTaskAssignment();
};

class Critical : public IBase
{
  friend class CriticalManager;
  Critical() = default;
  public:
    RobotClientsRobotStatus GetStatus() const override {return RobotClientsRobotStatus::Critical;}
};

class Charging : public IBase
{
  friend class Idle;
  Charging() = default;
  public:
    RobotClientsRobotStatus GetStatus() const override {return RobotClientsRobotStatus::Charging;}
    Idle ChargingComplete();
};

class Offline : public IBase
{
  public:
    RobotClientsRobotStatus GetStatus() const override {return RobotClientsRobotStatus::Offline;}
    Idle connected();
};

// Must be last
class CriticalManager
{
  private:
    static StateMachine savedStatus;
  public:
    static Critical EnterCritical(StateMachine currentStatus) {savedStatus = currentStatus; return Critical();}
    static StateMachine ExitCritical() {return savedStatus;}
};

}

#endif // ROBOT_STATUS_HPP