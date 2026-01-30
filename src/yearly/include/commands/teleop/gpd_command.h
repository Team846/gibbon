#pragma once

#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class DriveGPDCommand
    : public funkit::robot::GenericCommand<RobotContainer, DriveGPDCommand> {
public:
  DriveGPDCommand(RobotContainer& container);

  void OnInit() override;
  void Periodic() override;
  void OnEnd(bool interrupted) override;
  bool IsFinished() override;

private:
  RobotContainer& container_;
};
