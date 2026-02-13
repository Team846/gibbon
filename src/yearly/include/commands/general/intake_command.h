#pragma once

#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class IntakeCommand
    : public funkit::robot::GenericCommand<RobotContainer, IntakeCommand> {
public:
  IntakeCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
};