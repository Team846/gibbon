#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "funkit/base/Loggable.h"
#include "funkit/math/fieldpoints.h"
#include "funkit/robot/swerve/drivetrain.h"
#include "subsystems/robot_container.h"

namespace funkit::robot::swerve {

class WaitUntilClose
    : public frc2::CommandHelper<frc2::Command, WaitUntilClose>,
      public funkit::base::Loggable {
public:
  WaitUntilClose(
      DrivetrainSubsystem* drivetrain, funkit::math::FieldPoint target);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

protected:
  funkit::robot::swerve::DrivetrainSubsystem* drivetrain_;

  funkit::math::FieldPoint target_{};
};
}  // namespace funkit::robot::swerve