#pragma once

#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class AutoIntakeCommand
    : public funkit::robot::GenericCommand<RobotContainer, AutoIntakeCommand> {
public:
  AutoIntakeCommand(RobotContainer& container, HoptakeState target);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
  HoptakeState target_;
};