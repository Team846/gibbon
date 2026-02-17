#pragma once

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "pdcsu_control.h"

enum class TelescopeState { kStow, kIdleBackDrive, kDeployed };

struct TelescopeReadings {
  inch_t pos_;
  bool in_position_;
};

struct TelescopeTarget {
  TelescopeState target_state;
};

class TelescopeSubsystem
    : public funkit::robot::GenericSubsystem<TelescopeReadings,
          TelescopeTarget> {
public:
  TelescopeSubsystem();
  ~TelescopeSubsystem();

  void Setup() override;

  TelescopeTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroEncoders();

private:
  inch_t trgt_pos_{0_in_};

  funkit::control::HigherMotorController esc_;

  TelescopeReadings ReadFromHardware() override;

  void WriteToHardware(TelescopeTarget target) override;
};