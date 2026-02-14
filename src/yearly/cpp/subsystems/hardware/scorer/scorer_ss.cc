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

  RegisterPreference("passing/left_x", 0.0_in_);
  RegisterPreference("passing/left_y", 0.0_in_);
  RegisterPreference("passing/right_x", 0.0_in_);
  RegisterPreference("passing/right_y", 0.0_in_);

  RegisterPreference("point_blank/turret_angle", 0_deg_);
  RegisterPreference("point_blank/hood_angle", 85_deg_);
  RegisterPreference("point_blank/shooter_vel", 24_fps_);

  RegisterPreference("rotor_reset_loops", 20);
}

ScorerSuperstructure::~ScorerSuperstructure() = default;

void ScorerSuperstructure::Setup() {
  if (GetPreferenceValue_bool("init_turret")) {
    turret.InitByParent();
    turret.Setup();
  }
  if (GetPreferenceValue_bool("init_hood")) {
    hood.InitByParent();
    hood.Setup();
  }
  if (GetPreferenceValue_bool("init_shooter")) {
    shooter.InitByParent();
    shooter.Setup();
  }
  if (GetPreferenceValue_bool("init_dye_rotor")) {
    dye_rotor.InitByParent();
    dye_rotor.Setup();
  }
}

ScorerSSTarget ScorerSuperstructure::ZeroTarget() const {
  return ScorerSSTarget{{0_deg_, 0_degps_}, {80_deg_, 0_degps_}, 0_fps_,
      TrackingState::kTrack, false};
}

bool ScorerSuperstructure::VerifyHardware() {
  return turret.VerifyHardware() && hood.VerifyHardware() &&
         shooter.VerifyHardware() && dye_rotor.VerifyHardware();
}

ScorerSSReadings ScorerSuperstructure::ReadFromHardware() {
  ScorerSSReadings readings;

  readings.will_make_shot = turret.GetReadings().in_position_ &&
                            hood.GetReadings().in_position_ &&
                            shooter.GetReadings().is_spun_up;

  return readings;
}

void ScorerSuperstructure::AdjustTurret(bool cw) {
  (cw) ? turret_adjustment_ -= 0.5_deg_ : turret_adjustment_ += 0.5_deg_;
}

void ScorerSuperstructure::AdjustHood(bool up) {
  (up) ? hood_adjustment_ += 0.5_deg_ : hood_adjustment_ -= 0.5_deg_;
}

void ScorerSuperstructure::ClearAdjustments() {
  turret_adjustment_ = 0_deg_;
  hood_adjustment_ = 0_deg_;
}

void ScorerSuperstructure::WriteToHardware(ScorerSSTarget target) {
  if (target.tracking_state == TrackingState::kPointBlank) {
    hood.SetTarget(
        {GetPreferenceValue_unit_type<degree_t>("point_blank/hood_angle"),
            0_degps_});
    turret.SetTarget(
        {GetPreferenceValue_unit_type<degree_t>("point_blank/turret_angle"),
            0_degps_});
    shooter.SetTarget(
        {GetPreferenceValue_unit_type<fps_t>("point_blank/shooter_vel"), true});
  } else if (target.tracking_state == TrackingState::kTrack) {
    hood.SetTarget({target.hood_target});
    turret.SetTarget(target.turret_target);
    shooter.SetTarget({target.shooter_target, true});
  } else if (target.tracking_state == TrackingState::kLockTurret) {
    hood.SetTarget({target.hood_target});
    shooter.SetTarget({target.shooter_target, true});
  }

  if (last_shoot != target.shoot)
    rotor_reset_ctr = GetPreferenceValue_int("rotor_reset_loops");

  if (target.shoot) {
    if (rotor_reset_ctr > 0) {
      dye_rotor.SetTarget({DyeRotorState::kRotorReverse});
      rotor_reset_ctr--;
    } else {
      dye_rotor.SetTarget({DyeRotorState::kRotor84bps});
    }

  } else if (target.reverse_rotor) {
    dye_rotor.SetTarget({DyeRotorState::kRotorReverse});
  } else {
    dye_rotor.SetTarget({DyeRotorState::kRotorIdle});
  }

  last_shoot = target.shoot;
}