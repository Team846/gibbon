#include "commands/teleop/drive_to_climb_command.h"

#include <frc/DriverStation.h>

#include "funkit/math/fieldpoints.h"

DriveToClimbCommand::DriveToClimbCommand(RobotContainer& container,
    pdcsu::units::fps_t max_speed, pdcsu::units::fps2_t max_acceleration,
    pdcsu::units::fps2_t max_deceleration)
    : DriveToPointCommand{&container.drivetrain_, kClimbPointA, max_speed,
          max_acceleration, max_deceleration,
          funkit::robot::swerve::kRequireBearing},
      container_{container} {}

std::pair<funkit::math::FieldPoint, bool>
DriveToClimbCommand::GetTargetPoint() {
  bool isBlue =
      frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;

  auto pointA = kClimbPointA.mirror(isBlue);
  auto pointB = kClimbPointB.mirror(isBlue);

  auto pos = container_.drivetrain_.GetReadings().estimated_pose.position;
  auto distA = (pointA.point - pos).magnitude();
  auto distB = (pointB.point - pos).magnitude();

  auto closer = distA < distB ? pointA : pointB;

  return {closer, true};
}
