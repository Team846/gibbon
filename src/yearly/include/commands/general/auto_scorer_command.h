#pragma once

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>

#include "subsystems/robot_container.h"

frc2::InstantCommand AutoScorerCommand(
    RobotContainer& container, bool enable_shooting, bool enable_passing);
