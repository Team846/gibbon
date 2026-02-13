#pragma once

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

struct IntakeReadings {
  degps_t pos_;
};

struct IntakeTarget {
  degps_t pos_;
};

class IntakeSubsystem
    : public funkit::robot::GenericSubsystem<IntakeReadings, IntakeTarget> {
public:
  IntakeSubsystem();
  ~IntakeSubsystem();

  void Setup() override;

  IntakeTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroEncoders();

private:
  funkit::control::HigherMotorController esc_;

  IntakeReadings ReadFromHardware() override;

  void WriteToHardware(IntakeTarget target) override;
};
