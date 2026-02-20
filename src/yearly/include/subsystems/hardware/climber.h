#pragma once

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "pdcsu_control.h"

enum class ClimberState { kStow, kLevel1, kLevel3 };

struct ClimberReadings {
  degree_t pos_;
};

struct ClimberTarget {
  ClimberState target_state;
};

class ClimberSubsystem
    : public funkit::robot::GenericSubsystem<ClimberReadings, ClimberTarget> {
public:
  ClimberSubsystem();
  ~ClimberSubsystem();

  void Setup() override;

  ClimberTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroEncoders();

private:
  degree_t trgt_pos_{0_deg_};

  funkit::control::HigherMotorController esc_1_;
  funkit::control::HigherMotorController esc_2_;

  ClimberReadings ReadFromHardware() override;

  void WriteToHardware(ClimberTarget target) override;
};