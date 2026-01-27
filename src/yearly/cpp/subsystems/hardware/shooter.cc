#include "subsystems/hardware/shooter.h"

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

ShooterSubsystem::ShooterSubsystem()
    : GenericSubsystem("shooter"),
      esc_1_{base::SPARK_MAX_NEO550, ports::shooter_::kMotor1Params},
      esc_2_{base::SPARK_MAX_NEO550, ports::shooter_::kMotor2Params} {}

ShooterSubsystem::~ShooterSubsystem() = default;

void ShooterSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 22_A_,
      .smart_current_limit = 26_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.0, .kI = 0.0, .kD = 0.0, .kF = 0.0}};
  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::SPARK_MAX_NEO550);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  DefArmSys shooter_plant(
      def_bldc, 2, 2_rot_ / 1_rot_,
      [&](radian_t x, radps_t v) -> nm_t { return 0.0_Nm_; }, 0.001856_kgm2_,
      0.05_Nm_, 0.1_Nm_ / 1200_radps_, 20_ms_);

  esc_1_.Setup(genome_backup, shooter_plant);
  esc_2_.Setup(genome_backup, shooter_plant);

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
  FUNKIT_VERIFY(esc_1_.VerifyConnected(), ok, "Could not verify esc 1");
  FUNKIT_VERIFY(esc_2_.VerifyConnected(), ok, "Could not verify esc 2");
  return ok;
}

ShooterReadings ShooterSubsystem::ReadFromHardware() {
  radps_t vel =
      (esc_1_.GetVelocity<radps_t>() + esc_2_.GetVelocity<radps_t>()) / 2.0;
  fps_t vel_fps = vel * 2.0_in_ / 1_rad_;  // 2.0" wheel radius
  return ShooterReadings{vel_fps};
}

void ShooterSubsystem::WriteToHardware(ShooterTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");
  esc_1_.ModifyGenome(genome);
  esc_2_.ModifyGenome(genome);

  radps_t vel_radps = target.target_vel * 1_rad_ / 2.0_in_;

  esc_1_.WriteVelocityOnController(vel_radps);
  esc_2_.WriteVelocityOnController(vel_radps);
}