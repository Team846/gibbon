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

  RegisterPreference("passing/left_x", 70.0_in_);
  RegisterPreference("passing/left_y", 70.0_in_);
  RegisterPreference("passing/right_x", 247.8_in_);
  RegisterPreference("passing/right_y", 70.0_in_);

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

  turret.UpdateReadings();
  hood.UpdateReadings();
  shooter.UpdateReadings();
  dye_rotor.UpdateReadings();

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
  HoodTarget hood_trgt{0.0_deg_, 0.0_degps_};
  TurretTarget turret_trgt{0.0_deg_, 0.0_degps_};
  ShooterTarget shooter_trgt{0.0_fps_};
  DyeRotorTarget dye_rotor_trgt{DyeRotorState::kRotorIdle};

  if (target.tracking_state == TrackingState::kPointBlank) {
    hood_trgt = {
        GetPreferenceValue_unit_type<degree_t>("point_blank/hood_angle"),
        0_degps_};
    turret_trgt = {
        GetPreferenceValue_unit_type<degree_t>("point_blank/turret_angle"),
        0_degps_};
    shooter_trgt = {
        GetPreferenceValue_unit_type<fps_t>("point_blank/shooter_vel")};
  } else if (target.tracking_state == TrackingState::kTrack) {
    hood_trgt = target.hood_target;
    turret_trgt = target.turret_target;
    shooter_trgt = {target.shooter_target};
  } else if (target.tracking_state == TrackingState::kLockTurret) {
    hood_trgt = target.hood_target;
    shooter_trgt = {target.shooter_target};
  }

  if (last_shoot != target.shoot)
    rotor_reset_ctr = GetPreferenceValue_int("rotor_reset_loops");

  if (target.shoot) {
    if (rotor_reset_ctr > 0) {
      dye_rotor_trgt = {DyeRotorState::kRotorReverse};
      rotor_reset_ctr--;
    } else {
      dye_rotor_trgt = {DyeRotorState::kRotor84bps};
    }

  } else if (target.reverse_rotor) {
    dye_rotor_trgt = {DyeRotorState::kRotorReverse};
  } else {
    dye_rotor_trgt = {DyeRotorState::kRotorIdle};
  }

  hood.SetTarget(hood_trgt);
  turret.SetTarget(turret_trgt);
  shooter.SetTarget(shooter_trgt);
  dye_rotor.SetTarget(dye_rotor_trgt);

  hood.UpdateHardware();
  turret.UpdateHardware();
  shooter.UpdateHardware();
  dye_rotor.UpdateHardware();

  last_shoot = target.shoot;
}