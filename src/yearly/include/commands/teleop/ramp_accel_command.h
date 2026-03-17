#pragma once

#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class RampAccelCommand
    : public funkit::robot::GenericCommand<RobotContainer, RampAccelCommand> {
public:
  RampAccelCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
  double initial_timer;
  double accel_timer;
  bool start_accel;
};