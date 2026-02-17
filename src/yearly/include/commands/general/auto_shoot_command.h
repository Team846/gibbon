#pragma once

#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class AutoShootCommand
    : public funkit::robot::GenericCommand<RobotContainer, AutoShootCommand> {
public:
  AutoShootCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
};