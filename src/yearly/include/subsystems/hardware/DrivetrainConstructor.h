#pragma once

#include "funkit/base/Loggable.h"
#include "funkit/robot/swerve/drivetrain.h"

/*
DrivetrainConstructor

A class providing methods to aid construction of a DrivetrainSubsystem
object.
*/
class DrivetrainConstructor : public funkit::base::Loggable {
public:
  DrivetrainConstructor();

  funkit::robot::swerve::DrivetrainConfigs getDrivetrainConfigs();
};