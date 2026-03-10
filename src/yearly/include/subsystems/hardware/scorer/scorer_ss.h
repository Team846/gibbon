#pragma once

#include <frc/DigitalInput.h>

#include <cstdint>
#include <vector>

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

enum class TrackingState {
  kTrack,
  kPointBlank,
  kLockTurret,
};

struct ScorerSSReadings {
  bool will_make_shot;
};

struct ScorerSSTarget {
  TurretTarget turret_target;
  HoodTarget hood_target;
  fps_t shooter_target;
  TrackingState tracking_state;
  bool shoot;

  // Overrides
  bool reverse_rotor = false;
};

class ScorerSuperstructure
    : public funkit::robot::GenericSubsystem<ScorerSSReadings, ScorerSSTarget> {
public:
  ScorerSuperstructure();
  ~ScorerSuperstructure();

  TurretSubsystem turret;
  HoodSubsystem hood;
  ShooterSubsystem shooter;
  DyeRotorSubsystem dye_rotor;

  void Setup() override;

  ScorerSSTarget ZeroTarget() const override;

  void AdjustTurret(bool cw = false);
  void AdjustHood(bool up = false);
  void ClearAdjustments();

  bool VerifyHardware() override;

  void ZeroEncoders();

private:
  struct HysteresisWindow {
    std::vector<uint8_t> samples;
    int index = 0;
    int valid_count = 0;
    int true_count = 0;

    void Configure(int loops);
    void Push(bool sample);
    bool Full() const;
    double Fraction() const;
  };

  ScorerSSReadings ReadFromHardware() override;
  void WriteToHardware(ScorerSSTarget target) override;
  void ConfigureHysteresisWindows();

  bool last_shoot = false;
  bool will_make_shot_hysteresis_ = false;

  degree_t turret_adjustment_ = 0_deg_;
  degree_t hood_adjustment_ = 0_deg_;

  size_t rotor_reset_ctr = 0U;

  int on_window_loops_ = 0;
  int off_window_loops_ = 0;
  int off_recent_window_loops_ = 0;

  HysteresisWindow on_window_;
  HysteresisWindow off_window_;
  HysteresisWindow off_recent_window_;
};
