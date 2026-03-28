#include "subsystems/hardware/hoptake/pivot.h"

#include "funkit/control/calculators/CircuitResistanceCalculator.h"
#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

PivotSubsystem::PivotSubsystem()
    : GenericSubsystem("pivot"),
      esc_1_{base::TALON_FX_KRAKENX44, ports::pivot_::kPivotLeftParams},
      esc_2_{base::TALON_FX_KRAKENX44, ports::pivot_::kPivotRightParams} {
  RegisterPreference("pos_stow", -57.0_deg_);
  RegisterPreference("pos_intake", -76.0_deg_);
  RegisterPreference("pos_collapsed", -10.0_deg_);

  RegisterPreference("agigtate/on_loop_count", 10);
  RegisterPreference("agigtate/off_loop_count", 20);
}

PivotSubsystem::~PivotSubsystem() = default;

void PivotSubsystem::Setup() {
  if (!is_initialized()) return;

  MotorGenome genome_backup{.motor_current_limit = 200_A_,
      .smart_current_limit = 200_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = false,
      .gains = {.kP = 0.007, .kI = -0.03, .kD = 0.0, .kF = -0.12}};

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
      0.001044_kgm2_, 0.05_Nm_, 0.1_Nm_ / 1200_radps_, 20_ms_,
      funkit::control::calculators::CircuitResistanceCalculator::calculate(
          inch_t{50}, funkit::control::calculators::WireGauge::sixteen_gauge,
          0));

  esc_1_.Setup(genome_backup, pivot_plant);
  esc_2_.Setup(genome_backup, pivot_plant);

  esc_1_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});

  esc_2_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});

  esc_1_.SetPosition(radian_t{0});
  esc_2_.SetPosition(radian_t{0});
}

PivotTarget PivotSubsystem::ZeroTarget() const {
  return PivotTarget{PivotState::kStow};
}

void PivotSubsystem::ZeroSubsystem(bool at_hardstop) {
  if (!is_initialized()) {
    homed = true;
    return;
  };
  if (at_hardstop) {
    esc_1_.SetPosition(GetPreferenceValue_unit_type<degree_t>("pos_intake"));
    esc_2_.SetPosition(GetPreferenceValue_unit_type<degree_t>("pos_intake"));
  } else {
    esc_1_.SetPosition(radian_t{0});
    esc_2_.SetPosition(radian_t{0});
  }
  homed = true;
}

bool PivotSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(
      esc_1_.VerifyConnected(), ok, "Could not verify Left Pivot esc");
  FUNKIT_VERIFY(
      esc_2_.VerifyConnected(), ok, "Could not verify Right Pivot esc");
  return ok;
}

PivotReadings PivotSubsystem::ReadFromHardware() {
  degree_t current_pos = esc_1_.GetPosition<radian_t>();
  degree_t current_pos_2 = esc_2_.GetPosition<radian_t>();

  degree_t error = trgt_pos_ - current_pos;
  degree_t error_2 = trgt_pos_ - current_pos_2;
  Graph("error_left", error);
  Graph("error_right", error_2);

  // Graph("current", esc_.GetCurrent());

  return PivotReadings{current_pos};
}

void PivotSubsystem::WriteToHardware(PivotTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");
  esc_1_.ModifyGenome(genome);
  esc_2_.ModifyGenome(genome);

  if (target.target_state == PivotState::kStow) {
    trgt_pos_ = GetPreferenceValue_unit_type<degree_t>("pos_stow");
  } else if (target.target_state == PivotState::kCollapsed) {
    trgt_pos_ = GetPreferenceValue_unit_type<degree_t>("pos_collapsed");
  } else {
    trgt_pos_ = GetPreferenceValue_unit_type<degree_t>("pos_intake");
  }

  esc_1_.WritePosition(trgt_pos_);
  esc_2_.WritePosition(trgt_pos_);

  // if (trgt_pos_ < GetReadings().pos_) {
  //   esc_.WriteDC((trgt_pos_ - GetReadings().pos_).value() * genome.gains.kP +
  //                genome.gains.kI * u_sin(GetReadings().pos_) -
  //                genome.gains.kD * esc_.GetVelocity<radps_t>().value());
  // } else {
  //   esc_.WriteDC((trgt_pos_ - GetReadings().pos_).value() * genome.gains.kP +
  //                genome.gains.kF * u_sin(GetReadings().pos_) -
  //                genome.gains.kD * esc_.GetVelocity<radps_t>().value());
  // }
}