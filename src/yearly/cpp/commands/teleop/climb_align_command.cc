#include "commands/teleop/climb_align_command.h"

#include <frc/DriverStation.h>

#include "funkit/math/fieldpoints.h"


ClimbAlignCommand::ClimbAlignCommand(RobotContainer& container, funkit::math::FieldPoint target, pdcsu::units::fps_t max_speed,
      pdcsu::units::fps2_t max_acceleration, pdcsu::units::fps2_t max_deceleration, funkit::robot::swerve::DriveToPointFlags flags)
    : DriveToPointCommand{&container.drivetrain_, target, max_speed,
          max_acceleration, max_deceleration, flags} {}

std::pair<funkit::math::FieldPoint, bool> ClimbAlignCommand::GetTargetPoint() {
  bool isBlue = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
  return {kClimbPointRed.mirror(isBlue), true};
}