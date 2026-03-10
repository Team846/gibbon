#pragma once

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>

#include "subsystems/robot_container.h"

frc2::InstantCommand AutoIntakeCommand(
    RobotContainer& container, HoptakeState target);