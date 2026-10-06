#pragma once

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/control/base/current_logger.h"
#include "funkit/math/RampRateLimiter.h"
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
  double dye_rotor_pct_override = 0.0;
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

  void StartCurrentRecording(const std::string& filename) {
    current_logger_.StartRecording(filename);
  }
  bool StopCurrentRecording() { return current_logger_.StopRecording(); }
  bool IsCurrentRecording() const { return current_logger_.IsRecording(); }

private:
  radps_t getTargetRotorSpeed(DyeRotorState rotor_state);

  DyeRotorReadings ReadFromHardware() override;
  void WriteToHardware(DyeRotorTarget target) override;

  funkit::control::HigherMotorController esc_;

  DyeRotorState current_state;

  funkit::math::RampRateLimiter ramp_rate{};

  int reset_ctr_ = 0;
  int stall_ctr_ = 0;

  funkit::control::base::CurrentLogger current_logger_{"DyeRotorCurrent"};
};
