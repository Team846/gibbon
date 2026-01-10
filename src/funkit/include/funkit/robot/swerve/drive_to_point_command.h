#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "funkit/base/Loggable.h"
#include "funkit/math/fieldpoints.h"
#include "funkit/robot/swerve/drivetrain.h"
#include "pdcsu_units.h"

namespace funkit::robot::swerve {

enum DriveToPointFlags {
  kNone = 0,
  kLockToPoint = 1 << 0,
  kRequireBearing = 1 << 1,
  kNoTimeout = 1 << 2
};
inline DriveToPointFlags operator|(DriveToPointFlags a, DriveToPointFlags b) {
  return static_cast<DriveToPointFlags>(
      static_cast<int>(a) | static_cast<int>(b));
}

class DriveToPointCommand
    : public frc2::CommandHelper<frc2::Command, DriveToPointCommand>,
      public funkit::base::Loggable {
public:
  DriveToPointCommand(funkit::robot::swerve::DrivetrainSubsystem* drivetrain,
      funkit::math::FieldPoint target, pdcsu::units::fps_t max_speed,
      pdcsu::units::fps2_t max_acceleration,
      pdcsu::units::fps2_t max_deceleration, DriveToPointFlags flags = kNone);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

protected:
  funkit::robot::swerve::DrivetrainSubsystem* drivetrain_;

  virtual std::pair<funkit::math::FieldPoint, bool> GetTargetPoint() {
    return {{{pdcsu::units::inch_t{0}, pdcsu::units::inch_t{0}},
                pdcsu::units::degree_t{0}, pdcsu::units::fps_t{0}},
        false};
  };

  funkit::math::Vector2D start_point_;

private:
  pdcsu::units::fps_t max_speed_;
  pdcsu::units::fps2_t max_acceleration_;
  pdcsu::units::fps2_t max_deceleration_;

  funkit::math::FieldPoint target_;

  DriveToPointFlags flags_;

  pdcsu::units::second_t start_time_;
  pdcsu::units::second_t estimated_time_;

  pdcsu::units::second_t EstimateCompletionTime(pdcsu::units::inch_t distance,
      pdcsu::units::fps_t initial_velocity,
      pdcsu::units::fps_t final_velocity) const;
};

}  // namespace funkit::robot::swerve