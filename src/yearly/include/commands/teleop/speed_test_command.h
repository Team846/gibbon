#pragma once

#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class SpeedTestCommand
    : public funkit::robot::GenericCommand<RobotContainer, SpeedTestCommand> {
public:
  SpeedTestCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;
};