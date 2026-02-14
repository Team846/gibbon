#pragma once

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

struct TurretReadings {
  degree_t pos_;
  bool in_position_;
};

struct TurretTarget {
  degree_t pos_;
};

class TurretSubsystem
    : public funkit::robot::GenericSubsystem<TurretReadings, TurretTarget> {
public:
  TurretSubsystem();
  ~TurretSubsystem();

  void Setup() override;

  TurretTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroEncoders();

private:
  TurretReadings ReadFromHardware() override;

  funkit::control::HigherMotorController esc_;

  void WriteToHardware(TurretTarget target) override;
};
