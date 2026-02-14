#pragma once

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

enum class DyeRotorState {
  kRotor84bps,
  kRotorSlowFeed,
  kRotorReverse,
  kRotorIdle,
};
struct DyeRotorReadings {
  degps_t velocity_error;
};

struct DyeRotorTarget {
  DyeRotorState target_state;
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
  radps_t getTargetRotorSpeed(DyeRotorState rotor_state);

  DyeRotorReadings ReadFromHardware() override;
  void WriteToHardware(DyeRotorTarget target) override;

  funkit::control::HigherMotorController esc_;

  DyeRotorState current_state;
};
