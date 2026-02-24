#include "subsystems/hardware/scorer/shooter.h"

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

ShooterSubsystem::ShooterSubsystem()
    : GenericSubsystem("shooter"),
      esc_1_{base::TALON_FX_KRAKENX60, ports::shooter_::kShooter1Params},
      esc_2_{base::TALON_FX_KRAKENX60, ports::shooter_::kShooter2Params} {
  RegisterPreference("velocity_tolerance", 0.5_fps_);

  RegisterPreference("coast_down_tolerance", 5_fps_);
}

ShooterSubsystem::~ShooterSubsystem() = default;

void ShooterSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 120_A_,
      .smart_current_limit = 120_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.001, .kI = 0.0, .kD = 0.0, .kF = 0.22}};

  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::TALON_FX_KRAKENX60);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  DefLinearSys shooter_plant(def_bldc, 2,
      20_rot_ / (14.0 * 2.0 * 3.14159265358979323846 * kWheelRadius), 0.0_mps2_,
      1.5_lb_, 1.0_N_, 1.0_N_ / 628_radps_, 10_ms_);

  esc_1_.Setup(genome_backup, shooter_plant);
  auto genome2 = genome_backup;
  // genome2.follower_config = {ports::shooter_::kShooter1Params.can_id, true};
  esc_2_.Setup(genome2, shooter_plant);

  esc_1_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});
  esc_2_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});
  esc_1_.SetPosition(meter_t{0});
  esc_2_.SetPosition(meter_t{0});
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
  fps_t vel = (esc_1_.GetVelocity<mps_t>() + esc_2_.GetVelocity<mps_t>()) / 2.0;
  Graph("readings/velocity", vel);

  bool is_spun_up = u_abs(vel - GetTarget().target_vel) <
                    GetPreferenceValue_unit_type<fps_t>("velocity_tolerance");

  Graph("readings/is_spun_up", is_spun_up);

  return ShooterReadings{vel, is_spun_up};
}

void ShooterSubsystem::WriteToHardware(ShooterTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");

  esc_1_.ModifyGenome(genome);

  auto genome2 = genome;
  // genome2.follower_config = {ports::shooter_::kShooter1Params.can_id, true};
  esc_2_.ModifyGenome(genome2);

  Graph("debug/target", target.target_vel);

  Graph("debug/velocity_error", target.target_vel - GetReadings().vel);

  if ((target.target_vel < 5_fps_) &&
      (u_abs(GetReadings().vel) - u_abs(target.target_vel)) >=
          GetPreferenceValue_unit_type<fps_t>("coast_down_tolerance")) {
    esc_1_.WriteDC(0.0);
    esc_2_.WriteDC(0.0);
  }

  esc_1_.WriteVelocityOnController(target.target_vel);
  esc_2_.WriteVelocityOnController(
      target.target_vel);  // Function is no-op when esc_2_ is follower
}