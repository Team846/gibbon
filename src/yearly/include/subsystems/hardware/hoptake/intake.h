#pragma once

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/control/base/current_logger.h"
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

  void StartCurrentRecording(const std::string& filename) {
    current_logger_.StartRecording(filename);
  }
  bool StopCurrentRecording() { return current_logger_.StopRecording(); }
  bool IsCurrentRecording() const { return current_logger_.IsRecording(); }

private:
  funkit::control::HigherMotorController esc_;

  fps_t trgt_vel_{0.0_fps_};

  IntakeReadings ReadFromHardware() override;

  void WriteToHardware(IntakeTarget target) override;

  int reset_ctr_ = 0;
  int stall_ctr_ = 0;

  funkit::control::base::CurrentLogger current_logger_{"IntakeCurrent"};
};
