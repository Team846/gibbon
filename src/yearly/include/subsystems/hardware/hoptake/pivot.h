#pragma once

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

struct PivotReadings {
  degree_t pos_;
};

struct PivotTarget {
  degree_t pos_;
};

class PivotSubsystem
    : public funkit::robot::GenericSubsystem<PivotReadings, PivotTarget> {
public:
  PivotSubsystem();
  ~PivotSubsystem();

  void Setup() override;

  PivotTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroEncoders();

private:
  funkit::control::HigherMotorController esc_;

  PivotReadings ReadFromHardware() override;

  void WriteToHardware(PivotTarget target) override;
};
