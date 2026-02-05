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

struct ShooterReadings {
  fps_t vel;
  bool is_spun_up;
};

struct ShooterTarget {
  fps_t target_vel;
};

class ShooterSubsystem
    : public funkit::robot::GenericSubsystem<ShooterReadings, ShooterTarget> {
public:
  ShooterSubsystem();
  ~ShooterSubsystem();

  void Setup() override;

  ShooterTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  ShooterReadings ReadFromHardware() override;

  void WriteToHardware(ShooterTarget target) override;

  funkit::control::HigherMotorController esc_1_;
  funkit::control::HigherMotorController esc_2_;
};
