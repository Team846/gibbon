#pragma once

#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

enum struct IntakeState { kIdle, kIntake };

struct IntakeReadings {};

struct IntakeTarget {
  IntakeState state;
  double realintake;
};

class IntakeSubsystem
    : public funkit::robot::GenericSubsystem<IntakeReadings, IntakeTarget> {
public:
  IntakeSubsystem();
  ~IntakeSubsystem();

  void Setup() override;

  IntakeTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  IntakeReadings ReadFromHardware() override;

  void WriteToHardware(IntakeTarget target) override;

  funkit::control::HigherMotorController esc_;
  funkit::control::HigherMotorController intesc_;
};
