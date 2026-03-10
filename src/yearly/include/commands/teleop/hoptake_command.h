#pragma once

#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class HoptakeCommand
    : public funkit::robot::GenericCommand<RobotContainer, HoptakeCommand> {
public:
  HoptakeCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
};