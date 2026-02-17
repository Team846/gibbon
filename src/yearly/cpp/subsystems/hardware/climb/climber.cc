#include "subsystems/hardware/climb/climber.h"

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

ClimberSubsystem::ClimberSubsystem()
    : GenericSubsystem("climber"),
      esc_1_{base::SPARK_MAX_NEO, ports::climber_::kClimber1Params},
      esc_2_{base::SPARK_MAX_NEO, ports::climber_::kClimber2Params} {
  RegisterPreference("pos_stow", 0.0_deg_);
  RegisterPreference("pos_level1", 15.0_deg_);
  RegisterPreference("pos_level2", 90.0_deg_);
}

ClimberSubsystem::~ClimberSubsystem() = default;

void ClimberSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 35_A_,
      .smart_current_limit = 35_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};

  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::TALON_FX_KRAKENX60);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  // TODO: Fix
  DefArmSys climber_plant(
      def_bldc, 2, 500_rot_ / 1_rot_,
      [&](radian_t x, radps_t v) -> nm_t { return 0.0_Nm_; }, 0.001044_kgm2_,
      0.05_Nm_, 0.1_Nm_ / 1200_radps_, 20_ms_);

  esc_1_.Setup(genome_backup, climber_plant);
  esc_2_.Setup(genome_backup, climber_plant);

  esc_1_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});
  esc_2_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});

  esc_1_.SetPosition(radian_t{0});
  esc_2_.SetPosition(radian_t{0});
}

ClimberTarget ClimberSubsystem::ZeroTarget() const {
  return ClimberTarget{ClimberState::kStow};
}

bool ClimberSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_1_.VerifyConnected(), ok, "Could not verify Climber esc 1");
  FUNKIT_VERIFY(esc_2_.VerifyConnected(), ok, "Could not verify Climber esc 2");
  return ok;
}

ClimberReadings ClimberSubsystem::ReadFromHardware() {
  degree_t current_pos = esc_1_.GetPosition<radian_t>();

  degree_t error = trgt_pos_ - current_pos;
  Graph("error", error);

  return ClimberReadings{current_pos};
}

void ClimberSubsystem::WriteToHardware(ClimberTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");
  esc_1_.ModifyGenome(genome);
  esc_2_.ModifyGenome(genome);

  if (target.target_state == ClimberState::kStow) {
    trgt_pos_ = GetPreferenceValue_unit_type<degree_t>("pos_stow");
  } else if (target.target_state == ClimberState::kLevel1) {
    trgt_pos_ = GetPreferenceValue_unit_type<degree_t>("pos_level1");
  } else if (target.target_state == ClimberState::kLevel3) {
    trgt_pos_ = GetPreferenceValue_unit_type<degree_t>("pos_level3");
  }

  esc_1_.WritePosition(trgt_pos_);
  esc_2_.WritePosition(trgt_pos_);
}