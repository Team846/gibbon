#include "subsystems/hardware/scorer/dye_rotor.h"

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

DyeRotorSubsystem::DyeRotorSubsystem()
    : GenericSubsystem("DyeRotor"),
      esc_{base::TALON_FX_KRAKENX60, ports::dye_rotor_::kDyeRotorParams} {
  // Theoretical 375 RPM maximum speed
  RegisterPreference("speed_84bps", 300_rpm_);
  RegisterPreference("speed_slow_feed", 120_rpm_);
  RegisterPreference("speed_reverse", -200_rpm_);
  RegisterPreference("speed_idle", 0_rpm_);
}

radps_t DyeRotorSubsystem::getTargetRotorSpeed(DyeRotorState rotor_state) {
  switch (rotor_state) {
  case DyeRotorState::kRotor84bps:
    return GetPreferenceValue_unit_type<rpm_t>("speed_84bps");
  case DyeRotorState::kRotorSlowFeed:
    return GetPreferenceValue_unit_type<rpm_t>("speed_slow_feed");
  case DyeRotorState::kRotorReverse:
    return GetPreferenceValue_unit_type<rpm_t>("speed_reverse");
  default: return GetPreferenceValue_unit_type<rpm_t>("speed_idle");
  }
}

DyeRotorSubsystem::~DyeRotorSubsystem() = default;

void DyeRotorSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 90_A_,
      .smart_current_limit = 90_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};

  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::TALON_FX_KRAKENX60);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  DefArmSys DyeRotor_plant(
      def_bldc, 1, 16_rot_ / 1_rot_,
      [&](radian_t x, radps_t v) -> nm_t { return 0.0_Nm_; },
      3_lb_ * 7_in_ * 7_in_, 4.0_Nm_, 2.0_Nm_ / 628_radps_, 10_ms_);

  esc_.Setup(genome_backup, DyeRotor_plant);

  esc_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});

  esc_.SetPosition(radian_t{0});
}

DyeRotorTarget DyeRotorSubsystem::ZeroTarget() const {
  return DyeRotorTarget{DyeRotorState::kRotorIdle};
}

bool DyeRotorSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_.VerifyConnected(), ok, "Could not verify Dye Rotor esc");
  return ok;
}

DyeRotorReadings DyeRotorSubsystem::ReadFromHardware() {
  radps_t target_speed = getTargetRotorSpeed(current_state);
  radps_t error = target_speed - esc_.GetVelocity<radps_t>();

  return DyeRotorReadings{error};
}

void DyeRotorSubsystem::WriteToHardware(DyeRotorTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");

  esc_.ModifyGenome(genome);

  current_state = target.target_state;
  esc_.WriteVelocity(getTargetRotorSpeed(current_state));
}