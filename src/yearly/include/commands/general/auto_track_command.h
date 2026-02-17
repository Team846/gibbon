#pragma once

#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class AutoTrackCommand
    : public funkit::robot::GenericCommand<RobotContainer, AutoTrackCommand> {
public:
  AutoTrackCommand(RobotContainer& container, TrackingState target);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
  TrackingState target_;
};