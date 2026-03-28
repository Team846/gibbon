#pragma once

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

enum class PivotState { kStow, kIntake, kCollapsed };

struct PivotReadings {
  degree_t pos_;
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

  void ZeroSubsystem(bool at_hardstop = false);

  bool homed = false;

private:
  PivotReadings ReadFromHardware() override;

  void WriteToHardware(PivotTarget target) override;

  funkit::control::HigherMotorController esc_1_;
  funkit::control::HigherMotorController esc_2_;

  degree_t trgt_pos_{0_deg_};

  int ctr_agitate = 0;
};
