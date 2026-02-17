#include "lgdxrobot_cloud_adapter/RobotStatus.hpp"

// Idle State

RobotStatus::Charging RobotStatus::Idle::StartCharging()
{
  return RobotStatus::Charging();
}

RobotStatus::Running RobotStatus::Idle::TaskAssigned()
{
  return RobotStatus::Running();
}

RobotStatus::Paused RobotStatus::Idle::PauseTaskAssignment()
{
  return RobotStatus::Paused();
}

// Running State

RobotStatus::Idle RobotStatus::Running::TaskCompleted()
{
  return RobotStatus::Idle();
}

RobotStatus::Stuck RobotStatus::Running::NaviagtionStuck()
{
  return RobotStatus::Stuck();
}

RobotStatus::Aborting RobotStatus::Running::AbortTask()
{
  return RobotStatus::Aborting();
}

// Stuck State

RobotStatus::Idle RobotStatus::Stuck::TaskCompleted()
{
  return RobotStatus::Idle();
}

RobotStatus::Running RobotStatus::Stuck::Cleared()
{
  return RobotStatus::Running();
}

RobotStatus::Aborting RobotStatus::Stuck::AbortTask()
{
  return RobotStatus::Aborting();
}

// Aborting State

RobotStatus::Running RobotStatus::Aborting::TaskAssigned()
{
  return RobotStatus::Running();
}

RobotStatus::Idle RobotStatus::Aborting::TaskAborted()
{
  return RobotStatus::Idle();
}

// Paused State

RobotStatus::Idle RobotStatus::Paused::ResumeTaskAssignment()
{
  return RobotStatus::Idle();
}

// Charging State

RobotStatus::Idle RobotStatus::Charging::ChargingComplete()
{
  return RobotStatus::Idle();
}

// Offline State

RobotStatus::Idle RobotStatus::Offline::connected()
{
  return RobotStatus::Idle();
}