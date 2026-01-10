#pragma once

#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/swerve/drivetrain.h"
#include "subsystems/robot_container.h"

class LEDsLogic {
public:
  static void UpdateLEDs(RobotContainer* container);

  static void SetLEDsState(RobotContainer* container, LEDsState state);
  static void CoastingLEDs(RobotContainer* container, double percent);
};