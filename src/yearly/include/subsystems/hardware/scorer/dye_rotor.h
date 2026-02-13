#pragma once

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

struct DyeRotorReadings {
  degree_t pos_;
  bool has_balls_;
};

struct DyeRotorTarget {
  degree_t pos_;
};

class DyeRotorSubsystem
    : public funkit::robot::GenericSubsystem<DyeRotorReadings, DyeRotorTarget> {
public:
  DyeRotorSubsystem();
  ~DyeRotorSubsystem();

  void Setup() override;

  DyeRotorTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroEncoders();

private:
  DyeRotorReadings ReadFromHardware() override;

  funkit::control::HigherMotorController esc_;

  void WriteToHardware(DyeRotorTarget target) override;
};
