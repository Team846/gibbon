#include "subsystems/hardware/hoptake/intake.h"

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

IntakeSubsystem::IntakeSubsystem()
    : GenericSubsystem("Intake"),
      esc_{base::TALON_FX_KRAKENX44, ports::intake_::kIntakeParams} {
  RegisterPreference("speed_idle", 0.0_fps_);
  RegisterPreference("speed_intake", 25.0_fps_);
  RegisterPreference("speed_evac", -25.0_fps_);
}

IntakeSubsystem::~IntakeSubsystem() = default;

void IntakeSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 50_A_,
      .smart_current_limit = 50_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};

  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::TALON_FX_KRAKENX44);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  DefLinearSys intake_plant(def_bldc, 1,
      20_rot_ / 12_rot_ * 1_rad_ / 2.0625_in_, 0.0_mps2_, 1.0_kg_, 0.5_N_,
      0.5_N_ / 700_radps_, 20_ms_);

  esc_.Setup(genome_backup, intake_plant);

  esc_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});
  esc_.SetPosition(meter_t{0});
}

IntakeTarget IntakeSubsystem::ZeroTarget() const {
  return IntakeTarget{IntakeState::kIdle, 0.0_fps_};
}

bool IntakeSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_.VerifyConnected(), ok, "Could not verify Intake esc");
  return ok;
}

IntakeReadings IntakeSubsystem::ReadFromHardware() {
  fps_t velocity_ = esc_.GetVelocity<mps_t>();

  Graph("error", trgt_vel_ - velocity_);

  return IntakeReadings{velocity_};
}

void IntakeSubsystem::WriteToHardware(IntakeTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");

  esc_.ModifyGenome(genome);

  if (target.target_state == IntakeState::kIntake) {
    trgt_vel_ =
        GetPreferenceValue_unit_type<fps_t>("speed_intake") + target.dt_vel_;
  } else if (target.target_state == IntakeState::kEvac) {
    trgt_vel_ = GetPreferenceValue_unit_type<fps_t>("speed_evac");
  } else {
    trgt_vel_ = GetPreferenceValue_unit_type<fps_t>("speed_idle");
  }

  Graph("target_velocity", trgt_vel_);

  esc_.WriteVelocity(trgt_vel_);
}