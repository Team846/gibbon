#include "commands/teleop/climb_align_command.h"

ClimbAlignCommand::ClimbAlignCommand(RobotContainer& container,
    pdcsu::units::fps_t max_speed, pdcsu::units::fps_t lower_max_speed,
    pdcsu::units::fps2_t max_acceleration,
    pdcsu::units::fps2_t max_deceleration)
    : GenericCommandGroup<RobotContainer, ClimbAlignCommand,
          DriveToClimbCommand>{container, "climb_align_command",
          DriveToClimbCommand{
              container, max_speed, max_acceleration, max_deceleration}} {}