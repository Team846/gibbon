#include "subsystems/hardware/hoptake/pivot.h"

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

PivotSubsystem::PivotSubsystem()
    : GenericSubsystem("pivot"),
      esc_{base::TALON_FX_KRAKENX60, ports::pivot_::kPivotParams} {
  RegisterPreference("pos_stow", 0.0_deg_);
  RegisterPreference("pos_intake", 90.0_deg_);
  RegisterPreference("pos_collapsed", -10.0_deg_);

  RegisterPreference("agigtate/on_loop_count", 10);
  RegisterPreference("agigtate/off_loop_count", 20);
}

PivotSubsystem::~PivotSubsystem() = default;

void PivotSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 50_A_,
      .smart_current_limit = 50_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};

  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::TALON_FX_KRAKENX60);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  DefArmSys pivot_plant(
      def_bldc, 1, 50_rot_ / 12_rot_ * 60_rot_ / 18_rot_ * 64_rot_ / 18_rot_,
      [](radian_t x, radps_t v) -> nm_t {
        return nm_t{2.27_kg_ * 9.81_mps2_ * 0.23_m_ * u_cos(x)};
      },
      0.001044_kgm2_, 0.05_Nm_, 0.1_Nm_ / 1200_radps_, 20_ms_);

  esc_.Setup(genome_backup, pivot_plant);

  esc_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});

  esc_.SetPosition(radian_t{0});
}

PivotTarget PivotSubsystem::ZeroTarget() const {
  return PivotTarget{PivotState::kStow};
}

void PivotSubsystem::ZeroSubsystem() {
  esc_.SetPosition(radian_t{0});
  homed = true;
}

bool PivotSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_.VerifyConnected(), ok, "Could not verify Pivot esc");
  return ok;
}

PivotReadings PivotSubsystem::ReadFromHardware() {
  degree_t current_pos = esc_.GetPosition<radian_t>();

  degree_t error = trgt_pos_ - current_pos;
  Graph("error", error);

  // Graph("current", esc_.GetCurrent());

  return PivotReadings{current_pos};
}

void PivotSubsystem::WriteToHardware(PivotTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");
  esc_.ModifyGenome(genome);

  if (target.target_state == PivotState::kStow) {
    trgt_pos_ = GetPreferenceValue_unit_type<degree_t>("pos_stow");
  } else if (target.target_state == PivotState::kCollapsed) {
    trgt_pos_ = GetPreferenceValue_unit_type<degree_t>("pos_collapsed");
  } else {
    trgt_pos_ = GetPreferenceValue_unit_type<degree_t>("pos_intake");
  }

  if (trgt_pos_ < GetReadings().pos_) {
    esc_.WriteDC((trgt_pos_ - GetReadings().pos_).value() * genome.gains.kP +
                 genome.gains.kI * u_sin(GetReadings().pos_) -
                 genome.gains.kD * esc_.GetVelocity<radps_t>().value());
  } else {
    esc_.WriteDC((trgt_pos_ - GetReadings().pos_).value() * genome.gains.kP +
                 genome.gains.kF * u_sin(GetReadings().pos_) -
                 genome.gains.kD * esc_.GetVelocity<radps_t>().value());
  }
}