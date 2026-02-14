#pragma once

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

struct HoodReadings {
  degree_t pos_;
  bool in_position_;
};

struct HoodTarget {
  degree_t pos_;
};

class HoodSubsystem
    : public funkit::robot::GenericSubsystem<HoodReadings, HoodTarget> {
public:
  HoodSubsystem();
  ~HoodSubsystem();

  void Setup() override;

  HoodTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroEncoders();

private:
  funkit::control::HigherMotorController esc_;

  HoodReadings ReadFromHardware() override;

  void WriteToHardware(HoodTarget target) override;
};
