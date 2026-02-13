#include "subsystems/hardware/scorer/shooter.h"

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

ShooterSubsystem::ShooterSubsystem()
    : GenericSubsystem("shooter"),
      esc_1_{base::TALON_FX_KRAKENX60, ports::shooter_::kShooter1Params},
      esc_2_{base::TALON_FX_KRAKENX60, ports::shooter_::kShooter2Params} {
  RegisterPreference("velocity_tolerance", 0.25_fps_);
}

ShooterSubsystem::~ShooterSubsystem() = default;

void ShooterSubsystem::Setup() {
  MotorGenome genome_backup_esc_1{.motor_current_limit = 22_A_,
      .smart_current_limit = 26_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};

  MotorGenome genome_backup_esc_2 = genome_backup_esc_1;
  genome_backup_esc_2.follower_config = {
      ports::shooter_::kShooter1Params.can_id, true};

  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome_esc_1", genome_backup_esc_1);
  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome_esc_2", genome_backup_esc_2);

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::TALON_FX_KRAKENX60);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  // TODO: Fix
  DefArmSys shooter_plant(
      def_bldc, 2, 2_rot_ / 1_rot_,
      [&](radian_t x, radps_t v) -> nm_t { return 0.0_Nm_; }, 0.001044_kgm2_,
      0.05_Nm_, 0.1_Nm_ / 1200_radps_, 20_ms_);

  esc_1_.Setup(genome_backup_esc_1, shooter_plant);
  esc_2_.Setup(genome_backup_esc_2, shooter_plant);

  esc_1_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});
  esc_2_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});
  esc_1_.SetPosition(radian_t{0});
  esc_2_.SetPosition(radian_t{0});
}

ShooterTarget ShooterSubsystem::ZeroTarget() const {
  return ShooterTarget{0_fps_};
}

bool ShooterSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(esc_1_.VerifyConnected(), ok, "Could not verify Shooter esc 1");
  FUNKIT_VERIFY(esc_2_.VerifyConnected(), ok, "Could not verify Shooter esc 2");
  return ok;
}

ShooterReadings ShooterSubsystem::ReadFromHardware() {
  radps_t vel =
      (esc_1_.GetVelocity<radps_t>() + esc_2_.GetVelocity<radps_t>()) / 2.0;
  Graph("velnormal", vel);
  fps_t vel_fps = vel * kWheelRadius / 1_rad_;

  bool is_spun_up = pdcsu::units::u_abs(vel_fps - GetTarget().vel) <
                    GetPreferenceValue_unit_type<fps_t>("velocity_tolerance");

  return ShooterReadings{vel_fps, is_spun_up};
}

void ShooterSubsystem::WriteToHardware(ShooterTarget target) {
  auto genome_1_ =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome_esc_1");
  auto genome_2_ =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome_esc_2");

  esc_1_.ModifyGenome(genome_1_);
  esc_2_.ModifyGenome(genome_2_);

  radps_t vel_radps = target.vel * 1_rad_ / kWheelRadius;

  Graph("error", vel_radps * kWheelRadius / 1_rad_ - GetReadings().vel);
  Graph("vel", GetReadings().vel);
  esc_1_.WriteVelocityOnController(vel_radps);
  esc_2_.WriteVelocityOnController(
      vel_radps);  // following doesnt actually do anything
}