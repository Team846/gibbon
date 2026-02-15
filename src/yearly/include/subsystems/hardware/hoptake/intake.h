#pragma once

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

enum class IntakeState { kIdle, kIntake, kEvac };

struct IntakeReadings {
  fps_t vel_;
};

struct IntakeTarget {
  IntakeState target_state;
  fps_t dt_vel_;
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

  fps_t trgt_vel_{0.0_fps_};

  IntakeReadings ReadFromHardware() override;

  void WriteToHardware(IntakeTarget target) override;
};
