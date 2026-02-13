#pragma once

#include <frc/DriverStation.h>

#include "funkit/math/fieldpoints.h"
#include "funkit/robot/swerve/drive_to_point_command.h"
#include "subsystems/robot_container.h"

class ClimbAlignCommand : public funkit::robot::swerve::DriveToPointCommand {
public:
  ClimbAlignCommand(RobotContainer& container, funkit::math::FieldPoint target,
      pdcsu::units::fps_t max_speed, pdcsu::units::fps2_t max_acceleration,
      pdcsu::units::fps2_t max_deceleration,
      funkit::robot::swerve::DriveToPointFlags flags);

protected:
  std::pair<funkit::math::FieldPoint, bool> GetTargetPoint() override;

private:
};