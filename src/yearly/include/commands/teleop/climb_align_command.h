#pragma once

#include "commands/teleop/drive_to_climb_command.h"
#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class ClimbAlignCommand
    : public funkit::robot::GenericCommandGroup<RobotContainer,
          ClimbAlignCommand, DriveToClimbCommand> {
public:
  ClimbAlignCommand(RobotContainer& container, pdcsu::units::fps_t max_speed,
      pdcsu::units::fps_t lower_max_speed,
      pdcsu::units::fps2_t max_acceleration,
      pdcsu::units::fps2_t max_deceleration);
};