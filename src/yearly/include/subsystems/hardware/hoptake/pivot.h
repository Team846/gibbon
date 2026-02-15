#pragma once

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

enum class PivotState { kStow, kIntake, kAgitate };

struct PivotReadings {
  degree_t pos_;
  bool in_position_;
};

struct PivotTarget {
  PivotState target_state;
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

  degree_t trgt_pos_{0_deg_};

  PivotReadings ReadFromHardware() override;

  void WriteToHardware(PivotTarget target) override;
};
