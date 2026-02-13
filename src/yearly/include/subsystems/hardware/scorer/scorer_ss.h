#pragma once

#include <frc/DigitalInput.h>

#include "calculators/ShootingCalculator.h"
#include "calculators/TurretPositionCalculator.h"
#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/robot/swerve/drivetrain.h"
#include "funkit/wpilib/time.h"
#include "subsystems/hardware/scorer/dye_rotor.h"
#include "subsystems/hardware/scorer/hood.h"
#include "subsystems/hardware/scorer/shooter.h"
#include "subsystems/hardware/scorer/turret.h"

enum class ScorerState {
  kFollowHub,
  kPassingLeft,
  kPassingRight,
  kWithDT,
  kPointBlank
};

enum class ScorerOverrides {
  kNothing,
  kDisabled,
  kTurretNoSpin,
  kForceShoot,
  kOverrideAutoShoot
};

struct ScorerSSReadings {
  bool balls_feed_;
};

struct ScorerSSTarget {
  ShootingCalculatorOutputs shooting_outputs_;
  ScorerOverrides override_state_ = ScorerOverrides::kNothing;
  ScorerState state_ = ScorerState::kFollowHub;
};

class ScorerSuperstructure
    : public funkit::robot::GenericSubsystem<ScorerSSReadings, ScorerSSTarget> {
public:
  ScorerSuperstructure();
  ~ScorerSuperstructure();

  void Setup() override;

  ScorerSSTarget ZeroTarget() const override;

  TurretSubsystem turret;
  HoodSubsystem hood;
  ShooterSubsystem shooter;
  DyeRotorSubsystem dye_rotor;

  bool HasReachedShooter(degps_t vel);
  bool HasReachedHood(degree_t pos);
  bool HasReachedTurret(degree_t pos);

  void AdjustTurret(bool cw);
  void AdjustHood(bool up);
  void ClearAdjustments();

  ScorerState GetCurrState() const { return current_state_; }

  bool VerifyHardware() override;

  void ZeroEncoders();

private:
  ScorerSSReadings ReadFromHardware() override;

  frc::DigitalInput dye_rotor_dds_{5};
  int dye_rotor_counter_{0};

  ScorerState current_state_{ScorerState::kFollowHub};

  degree_t turret_adjustment_ = 0_deg_;
  degree_t hood_adjustment_ = 0_deg_;

  void WriteToHardware(ScorerSSTarget target) override;
};
