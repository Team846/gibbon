#include "subsystems/hardware/climb/telescope.h"

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

TelescopeSubsystem::TelescopeSubsystem()
    : GenericSubsystem("telescope"),
      esc_{base::SPARK_MAX_NEO, ports::telescope_::kTelescopeParams} {
  RegisterPreference("pos_stow", 0.0_in_);
  RegisterPreference("pos_deployed", 9.0_in_);
  RegisterPreference("pos_tolerance", 0.5_in_);
}

TelescopeSubsystem::~TelescopeSubsystem() = default;

void TelescopeSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 35_A_,
      .smart_current_limit = 35_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = false,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};

  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::SPARK_MAX_NEO);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  // TODO: Fix
  DefLinearSys telescope_plant(
      def_bldc, 1, 12_rot_ / 20_in_, 0.0_mps2_, 1.0_kg_,
      0.5_N_, 0.5_N_ / 700_radps_, 20_ms_);

  esc_.Setup(genome_backup, telescope_plant);

  esc_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});

  esc_.SetPosition(radian_t{0});
}

TelescopeTarget TelescopeSubsystem::ZeroTarget() const {
  return TelescopeTarget{TelescopeState::kStow};
}

bool TelescopeSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_.VerifyConnected(), ok, "Could not verify Telescope esc");
  return ok;
}

TelescopeReadings TelescopeSubsystem::ReadFromHardware() {
  inch_t curr_pos = esc_.GetPosition<inch_t>();
  
  inch_t error = trgt_pos_ - curr_pos;
  Graph("error", error);
  
  bool in_pos = u_abs(error) < GetPreferenceValue_unit_type<inch_t>("pos_tolerance");

  return TelescopeReadings{curr_pos, in_pos};
}

void TelescopeSubsystem::WriteToHardware(TelescopeTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");
  esc_.ModifyGenome(genome);

  if (target.target_state == TelescopeState::kStow) {
    trgt_pos_ = GetPreferenceValue_unit_type<inch_t>("pos_stow");
  } else if (target.target_state == TelescopeState::kDeployed) {
    trgt_pos_ = GetPreferenceValue_unit_type<inch_t>("pos_deployed");
  }

  if (target.target_state == TelescopeState::kIdleBackDrive) {
    esc_.WriteDC(0.0);
  } else {
    esc_.WritePosition(trgt_pos_);
  }
}