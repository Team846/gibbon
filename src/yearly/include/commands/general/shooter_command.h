#pragma once

#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class ShooterCommand
    : public funkit::robot::GenericCommand<RobotContainer, ShooterCommand> {
public:
  ShooterCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
  funkit::math::FieldPoint target_position{{0_in_, 0_in_}, 0_deg_, 0_fps_};
};