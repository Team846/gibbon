#include "subsystems/hardware/scorer/shooter.h"

#include <filesystem>
#include <fstream>

#include "funkit/control/config/genome.h"
#include "ports.h"

using namespace funkit::control;
using namespace funkit::control::config;

ShooterSubsystem::ShooterSubsystem()
    : GenericSubsystem("shooter"),
      esc_1_{base::TALON_FX_KRAKENX60, ports::shooter_::kShooter1Params},
      esc_2_{base::TALON_FX_KRAKENX60, ports::shooter_::kShooter2Params} {
  RegisterPreference("velocity_tolerance", 5.0_fps_);

  RegisterPreference("coast_down_tolerance", 5_fps_);

  RegisterPreference("accel_factor", 3.0);
  RegisterPreference("accel_alpha", 0.3);

  RegisterPreference("ramp_rate", 40.0);
}

ShooterSubsystem::~ShooterSubsystem() = default;

void ShooterSubsystem::Setup() {
  MotorGenome genome_backup{.motor_current_limit = 140_A_,
      .smart_current_limit = 140_A_,
      .voltage_compensation = 12_V_,
      .brake_mode = true,
      .gains = {.kP = 0.45, .kI = 0.0, .kD = 0.0, .kF = 0.118}};

  funkit::control::config::SubsystemGenomeHelper::CreateGenomePreferences(
      *this, "genome", genome_backup);

  auto motor_specs =
      base::MotorSpecificationPresets::get(base::TALON_FX_KRAKENX60);

  DefBLDC def_bldc(motor_specs.stall_current, motor_specs.free_current,
      motor_specs.stall_torque, motor_specs.free_speed, 12_V_);

  DefLinearSys shooter_plant(def_bldc, 2,
      20_rot_ / (14.0 * 2.0 * 3.14159265358979323846 * kWheelRadius), 0.0_mps2_,
      0.3_lb_, 1.0_N_, 1.0_N_ / 628_radps_, 10_ms_);

  esc_1_.Setup(genome_backup, shooter_plant);
  auto genome2 = genome_backup;
  genome2.follower_config = {ports::shooter_::kShooter1Params.can_id, false};
  esc_2_.Setup(genome2, shooter_plant);

  esc_1_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame,
          StatusFrame::kLeader},
      ms_t{20}, ms_t{5}, ms_t{5}, ms_t{20});
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
  fps_t vel =
      esc_1_.GetVelocity<mps_t>();  //(esc_1_.GetVelocity<mps_t>() +
                                    // esc_2_.GetVelocity<mps_t>()) / 2.0;
  Graph("readings/velocity", vel);

  bool is_spun_up = u_abs(vel - GetTarget().target_vel) <
                    GetPreferenceValue_unit_type<fps_t>("velocity_tolerance");

  if (filling_graph) {
    errors_graph[errors_graph_pos] =
        ((vel - GetTarget().target_vel) / vel).value();
    errors_graph_pos++;
    if (errors_graph_pos >= 280) {
      std::string dir = "/home/lvuser/shooter_errors.csv";
      std::ofstream file(dir, std::ios::out | std::ios::trunc);
      if (file.is_open()) {
        file << "index,error" << std::endl;
        for (size_t i = 0; i < 280; ++i) {
          file << i << "," << errors_graph[i] << std::endl;
        }
        file.close();
        Log("Wrote shooter errors_graph to {}", dir);
      } else {
        Warn("Failed to open {} for writing", dir);
      }
      errors_graph_pos = 0;
      filling_graph = false;
    }
  }

  Graph("readings/is_spun_up", is_spun_up);

  return ShooterReadings{vel, is_spun_up};
}

void ShooterSubsystem::StartGraph() { filling_graph = true; }

void ShooterSubsystem::WriteToHardware(ShooterTarget target) {
  auto genome =
      funkit::control::config::SubsystemGenomeHelper::LoadGenomePreferences(
          *this, "genome");

  esc_1_.ModifyGenome(genome);

  auto genome2 = genome;
  genome2.follower_config = {ports::shooter_::kShooter1Params.can_id, false};
  esc_2_.ModifyGenome(genome2);

  Graph("debug/target", target.target_vel);

  Graph("debug/velocity_error", target.target_vel - GetReadings().vel);

  fps2_t accel_inst = 0.0_fps2_;

  bool apply_ramp_limiter = target.target_vel < GetReadings().vel &&
                            u_abs(target.target_vel) < 22_fps_;

  if (last_time_ > 0.0_ms_) {
    auto dt = u_max(9.0_ms_, (funkit::wpilib::CurrentFPGATime() - last_time_));
    accel_inst = (target.target_vel - last_vel_) / dt *
                 GetPreferenceValue_double("accel_factor");
  }
  last_vel_ = target.target_vel;
  last_time_ = funkit::wpilib::CurrentFPGATime();

  double accel_alpha = GetPreferenceValue_double("accel_alpha");
  if (accel_alpha < 0.0) { accel_alpha = 0.0; }
  if (accel_alpha > 1.0) { accel_alpha = 1.0; }

  accel_est_ = fps2_t{accel_alpha * accel_inst.to_base() +
                      (1.0 - accel_alpha) * accel_est_.to_base()};

  Graph("debug/accel_inst", accel_inst);
  Graph("debug/accel", accel_est_);

  target.target_vel += accel_est_ * 0.1_s_;

  fps_t limited_target_vel = fps_t(ramp_rate.limit(
      target.target_vel.value(), GetPreferenceValue_double("ramp_rate")));

  if (apply_ramp_limiter) { target.target_vel = limited_target_vel; }

  if ((target.target_vel < 5_fps_) &&
      (u_abs(GetReadings().vel) - u_abs(target.target_vel)) >=
          GetPreferenceValue_unit_type<fps_t>("coast_down_tolerance")) {
    esc_1_.WriteDC(0.0);
    esc_2_.WriteDC(0.0);
  }

  esc_1_.WriteVelocityOnController(target.target_vel);
  if (frc::RobotBase::IsSimulation()) {
    esc_2_.WriteVelocityOnController(
        target.target_vel);  // Function is no-op (IRL) when esc_2_ is follower
  }
}