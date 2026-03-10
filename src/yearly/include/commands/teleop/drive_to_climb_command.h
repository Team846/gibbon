#pragma once

#include <frc/DriverStation.h>

#include "funkit/math/fieldpoints.h"
#include "funkit/robot/swerve/drive_to_point_command.h"
#include "subsystems/robot_container.h"

class DriveToClimbCommand : public funkit::robot::swerve::DriveToPointCommand {
public:
  DriveToClimbCommand(RobotContainer& container, pdcsu::units::fps_t max_speed,
      pdcsu::units::fps2_t max_acceleration,
      pdcsu::units::fps2_t max_deceleration);

protected:
  std::pair<funkit::math::FieldPoint, bool> GetTargetPoint() override;

private:
  RobotContainer& container_;

  funkit::math::FieldPoint kClimbPointA{
      {pdcsu::units::inch_t{72}, pdcsu::units::inch_t{200}},
      pdcsu::units::degree_t{0}, pdcsu::units::fps_t{0}};

  funkit::math::FieldPoint kClimbPointB{
      {pdcsu::units::inch_t{72}, pdcsu::units::inch_t{450}},
      pdcsu::units::degree_t{0}, pdcsu::units::fps_t{0}};
};
