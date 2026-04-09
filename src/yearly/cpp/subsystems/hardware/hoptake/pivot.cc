#include "subsystems/hardware/hoptake/pivot.h"

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

PivotSubsystem::PivotSubsystem()
    : GenericSubsystem("pivot"),
      esc_{base::TALON_FX_KRAKENX44, ports::pivot_::kPivotParams},
      esc2_{base::TALON_FX_KRAKENX44, ports::pivot_::kPivot2Params} {
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
      .gains = {.kP = 0.007, .kI = -0.03, .kD = 0.0, .kF = -0.03}};

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
  esc2_.Setup(genome_backup, pivot_plant);

  esc_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame,
          StatusFrame::kLeader},
      ms_t{20}, ms_t{5}, ms_t{5}, ms_t{20});

  esc_.SetPosition(radian_t{0});

  esc2_.EnableStatusFrames(
      {StatusFrame::kPositionFrame}, ms_t{20}, ms_t{5}, ms_t{5}, ms_t{20});
  esc2_.SetPosition(radian_t{0});
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
    esc_.SetPosition(GetPreferenceValue_unit_type<degree_t>("pos_intake"));
    esc2_.SetPosition(GetPreferenceValue_unit_type<degree_t>("pos_intake"));
  } else {
    esc_.SetPosition(radian_t{0});
    esc2_.SetPosition(radian_t{0});
  }
  homed = true;
}

bool PivotSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_.VerifyConnected(), ok, "Could not verify Pivot esc");
  FUNKIT_VERIFY(esc2_.VerifyConnected(), ok, "Could not verify Pivot2 esc");
  return ok;
}

PivotReadings PivotSubsystem::ReadFromHardware() {
  degree_t current_pos = esc_.GetPosition<radian_t>();
  degree_t pos2 = esc2_.GetPosition<radian_t>();

  degree_t error = trgt_pos_ - current_pos;
  Graph("error", error, true);

  Graph("pos", degree_t{current_pos});
  Graph("pos2", degree_t{pos2});

  // Graph("current", esc_.GetCurrent());

  return PivotReadings{current_pos, pos2};
}

void PivotSubsystem::WriteToHardware(PivotTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");
  esc_.ModifyGenome(genome);
  esc2_.ModifyGenome(genome);

  if (target.force_down) {
    esc_.WriteDC(-0.05);
    esc2_.WriteDC(-0.05);
    return;
  }

  if (target.target_state == PivotState::kStow) {
    trgt_pos_ = GetPreferenceValue_unit_type<degree_t>("pos_stow");
  } else if (target.target_state == PivotState::kCollapsed) {
    trgt_pos_ = GetPreferenceValue_unit_type<degree_t>("pos_collapsed");
  } else {
    trgt_pos_ = GetPreferenceValue_unit_type<degree_t>("pos_intake");
  }

  if (u_abs(GetReadings().pos_ - GetReadings().pos2_) > 5_deg_) {
    degree_t esc_1_dist_to_target = u_abs(GetReadings().pos_ - trgt_pos_);
    degree_t esc_2_dist_to_target = u_abs(GetReadings().pos2_ - trgt_pos_);
    if (esc_1_dist_to_target < esc_2_dist_to_target) {
      ctr_impact_follow_a = 25;
    } else {
      ctr_impact_follow_b = 25;
    }
  }

  degree_t target_pos_a = trgt_pos_;
  degree_t target_pos_b = trgt_pos_;

  // if (ctr_impact_follow_a > 0) {
  //   target_pos_a = GetReadings().pos2_;
  //   ctr_impact_follow_a--;
  // }
  // if (ctr_impact_follow_b > 0) {
  //   target_pos_b = GetReadings().pos_;
  //   ctr_impact_follow_b--;
  // }

  esc_.WriteDC((target_pos_a - GetReadings().pos_).value() * genome.gains.kP +
               genome.gains.kF * u_sin(GetReadings().pos_) -
               genome.gains.kD * esc_.GetVelocity<radps_t>().value());
  esc2_.WriteDC((target_pos_b - GetReadings().pos2_).value() * genome.gains.kP +
                genome.gains.kF * u_sin(GetReadings().pos2_) -
                genome.gains.kD * esc2_.GetVelocity<radps_t>().value());
}