#include "subsystems/hardware/scorer/scorer_ss.h"

#include <algorithm>

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

  RegisterPreference("will_make_shot_hysteresis/on_percent", 70.0);
  RegisterPreference("will_make_shot_hysteresis/on_window_loops", 40);
  RegisterPreference("will_make_shot_hysteresis/off_percent", 50.0);
  RegisterPreference("will_make_shot_hysteresis/off_window_loops", 20);
  RegisterPreference("will_make_shot_hysteresis/off_recent_window_loops", 10);

  RegisterPreference("rotor_reset_loops", 10);
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

void ScorerSuperstructure::HysteresisWindow::Configure(int loops) {
  samples.assign(std::max(1, loops), 0U);
  index = 0;
  valid_count = 0;
  true_count = 0;
}

void ScorerSuperstructure::HysteresisWindow::Push(bool sample) {
  if (samples.empty()) { return; }

  const uint8_t sample_value = sample ? 1U : 0U;
  if (valid_count < static_cast<int>(samples.size())) {
    valid_count++;
  } else {
    true_count -= samples[index];
  }

  samples[index] = sample_value;
  true_count += sample_value;
  index = (index + 1) % static_cast<int>(samples.size());
}

bool ScorerSuperstructure::HysteresisWindow::Full() const {
  return valid_count >= static_cast<int>(samples.size());
}

double ScorerSuperstructure::HysteresisWindow::Fraction() const {
  if (valid_count <= 0) { return 0.0; }
  return static_cast<double>(true_count) / static_cast<double>(valid_count);
}

void ScorerSuperstructure::ConfigureHysteresisWindows() {
  const int on_window_loops = std::max(
      1, GetPreferenceValue_int("will_make_shot_hysteresis/on_window_loops"));
  const int off_window_loops = std::max(
      1, GetPreferenceValue_int("will_make_shot_hysteresis/off_window_loops"));
  const int off_recent_window_loops =
      std::max(1, GetPreferenceValue_int(
                      "will_make_shot_hysteresis/off_recent_window_loops"));

  if (on_window_loops == on_window_loops_ &&
      off_window_loops == off_window_loops_ &&
      off_recent_window_loops == off_recent_window_loops_) {
    return;
  }

  on_window_loops_ = on_window_loops;
  off_window_loops_ = off_window_loops;
  off_recent_window_loops_ = off_recent_window_loops;

  on_window_.Configure(on_window_loops_);
  off_window_.Configure(off_window_loops_);
  off_recent_window_.Configure(off_recent_window_loops_);
}

ScorerSSReadings ScorerSuperstructure::ReadFromHardware() {
  ScorerSSReadings readings;

  turret.UpdateReadings();
  hood.UpdateReadings();
  shooter.UpdateReadings();
  dye_rotor.UpdateReadings();

  const bool raw_will_make_shot = turret.GetReadings().in_position_ &&
                                  hood.GetReadings().in_position_ &&
                                  shooter.GetReadings().is_spun_up;

  ConfigureHysteresisWindows();
  on_window_.Push(raw_will_make_shot);

  const bool off_sample = !raw_will_make_shot;
  off_window_.Push(off_sample);
  off_recent_window_.Push(off_sample);

  const double on_threshold = std::clamp(
      GetPreferenceValue_double("will_make_shot_hysteresis/on_percent") / 100.0,
      0.0, 1.0);
  const double off_threshold = std::clamp(
      GetPreferenceValue_double("will_make_shot_hysteresis/off_percent") /
          100.0,
      0.0, 1.0);

  if (!will_make_shot_hysteresis_) {
    if (on_window_.Full() && on_window_.Fraction() >= on_threshold) {
      will_make_shot_hysteresis_ = true;
    }
  } else if (off_window_.Full() && off_recent_window_.Full() &&
             off_window_.Fraction() >= off_threshold &&
             off_recent_window_.Fraction() > off_threshold) {
    will_make_shot_hysteresis_ = false;
  }

  readings.will_make_shot = will_make_shot_hysteresis_;

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