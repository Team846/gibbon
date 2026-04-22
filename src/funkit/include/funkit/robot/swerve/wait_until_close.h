#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "funkit/base/Loggable.h"
#include "funkit/math/fieldpoints.h"
#include "funkit/robot/swerve/drivetrain.h"
#include "subsystems/robot_container.h"

namespace funkit::robot::swerve {

/**
 * WaitUntilClose
 *
 * A class which inherits from CommandHelper and Loggable, acting as a command.
 * Returns true when distance from current_estimated_pose is less than a certain
 * distance.
 */
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