#include "subsystems/hardware/scorer/scorer_ss.h"

#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

ScorerSuperstructure::ScorerSuperstructure()
    : GenericSubsystem("scorer_ss"), turret(), hood(), shooter(), dye_rotor() {
  RegisterPreference("init_turret", true);
  RegisterPreference("init_hood", true);
  RegisterPreference("init_shooter", true);
  RegisterPreference("init_dye_rotor", true);

  RegisterPreference("dye_rotor_balless_thresh", 50);
  RegisterPreference("turret_adjustment", 0_deg_);
}

ScorerSuperstructure::~ScorerSuperstructure() = default;

void ScorerSuperstructure::Setup() {
  if (GetPreferenceValue_bool("init_turret")) {
    turret.Init();
    turret.Setup();
  }
  if (GetPreferenceValue_bool("init_hood")) {
    hood.Init();
    hood.Setup();
  }
  if (GetPreferenceValue_bool("init_shooter")) {
    shooter.Init();
    shooter.Setup();
  }
  if (GetPreferenceValue_bool("init_dye_rotor")) {
    dye_rotor.Init();
    dye_rotor.Setup();
  }
}

ScorerSSTarget ScorerSuperstructure::ZeroTarget() const {
  return ScorerSSTarget{};
}

bool ScorerSuperstructure::VerifyHardware() {
  return turret.VerifyHardware() && hood.VerifyHardware() &&
         shooter.VerifyHardware() && dye_rotor.VerifyHardware();
}

ScorerSSReadings ScorerSuperstructure::ReadFromHardware() {
  ScorerSSReadings readings;
  (!dye_rotor_dds_.Get()) ? dye_rotor_counter_++ : dye_rotor_counter_ = 0;

  if (dye_rotor_counter_ > GetPreferenceValue_int("dye_rotor_balless_thresh")) {
    readings.balls_feed_ = false;
  } else {
    readings.balls_feed_ = true;
  }

  return readings;
}

void ScorerSuperstructure::AdjustTurret(bool cw) {
  if (cw)
    turret_adjustment_ -= 0.5_deg;
  else
    turret_adjustment_ += 0.5_deg;
}

void ScorerSuperstructure::AdjustHood(bool up) {
  if (up)
    hood_adjustment_ += 0.5_deg;
  else
    hood_adjustment_ -= 0.5_deg;
}

void ScorerSuperstructure::ClearAdjustments() {
  turret_adjustment_ = 0_deg_;
  hood_adjustment_ = 0_deg_;
}

void ScorerSuperstructure::WriteToHardware(ScorerSSTarget target) {
  // check for overrides
  if (target.override_state_ != ScorerOverrides::kNothing) {
    if (target.override_state_ == ScorerOverrides::kTurretNoSpin) {
      current_state_ = ScorerState::kWithDT;
      // ignore turret assume drivetrain aligned
      // hood.SetTarget({target.shooting_outputs_.hood_pos});
      shooter.SetTarget({target.shooting_outputs_.shooter_vel});
    } else if (target.override_state_ == ScorerOverrides::kDisabled) {
      return;
    }
  } else {
    if (target.shooting_outputs_.is_valid) {
      turret.SetTarget(
          {target.shooting_outputs_.aim_angle + turret_adjustment_});
      hood.SetTarget({target.shooting_outputs_.hood_pos + hood_adjustment_});
      shooter.SetTarget({target.shooting_outputs_.shooter_vel});
      current_state_ = ScorerState::kFollowHub;
    } else {
      turret.SetTargetZero();
      hood.SetTargetZero();
      shooter.SetTargetZero();
      current_state_ = ScorerState::kFollowHub;
    }
  }
}