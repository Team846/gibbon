#include "funkit/robot/swerve/swerve_module.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/torque.h>

#include <optional>
#include <thread>

#include "frc/RobotBase.h"
#include "funkit/control/config/genome.h"
#include "funkit/math/collection.h"
#include "pdcsu_units.h"

namespace funkit::robot::swerve {

SwerveModuleSubsystem::SwerveModuleSubsystem(Loggable& parent,
    SwerveModuleUniqueConfig unique_config,
    SwerveModuleCommonConfig common_config)
    : funkit::robot::GenericSubsystem<SwerveModuleReadings, SwerveModuleTarget>(
          parent, unique_config.loc),
      avg_resistance_{common_config.avg_resistance},
      circuit_resistance_{common_config.circuit_resistance},
      motor_types_{common_config.motor_types},
      drive_params_{getMotorParams(unique_config, common_config).first},
      steer_params_{getMotorParams(unique_config, common_config).second},
      drive_{common_config.motor_types, drive_params_},
      steer_{common_config.motor_types, steer_params_},
      drive_plant_{common_config.drive_plant},
      steer_plant_{common_config.steer_plant},
      cancoder_{unique_config.cancoder_id, common_config.bus},
      steer_load_factor_{common_config.steer_load_factor} {
  cancoder_.OptimizeBusUtilization();
  cancoder_.GetAbsolutePosition().SetUpdateFrequency(20_Hz);

  RegisterPreference("cancoder_offset_", degree_t{0.0});

  auto motor_specs = funkit::control::base::MotorSpecificationPresets::get(
      common_config.motor_types);
  auto free_speed_rpm = motor_specs.free_speed;
  auto drive_reduction_pdcsu = common_config.drive_reduction;
  auto free_speed_rpm_value = free_speed_rpm.value();
  auto drive_reduction_ft_per_rot = drive_reduction_pdcsu.value();
  auto max_speed_ft_per_min = free_speed_rpm_value * drive_reduction_ft_per_rot;
  max_speed_ = fps_t{max_speed_ft_per_min / 60.0};
}

std::pair<funkit::control::config::MotorConstructionParameters,
    funkit::control::config::MotorConstructionParameters>
SwerveModuleSubsystem::getMotorParams(SwerveModuleUniqueConfig unique_config,
    SwerveModuleCommonConfig common_config) {
  funkit::control::config::MotorConstructionParameters drive_params =
      common_config.drive_params;
  funkit::control::config::MotorConstructionParameters steer_params =
      common_config.steer_params;

  drive_params.can_id = unique_config.drive_id;
  steer_params.can_id = unique_config.steer_id;

  drive_params.bus = common_config.bus;
  steer_params.bus = common_config.bus;

  return {drive_params, steer_params};
}

void SwerveModuleSubsystem::SetDriveGenome(
    funkit::control::config::MotorGenome genome) {
  drive_genome_ = genome;
}

void SwerveModuleSubsystem::SetSteerGenome(
    funkit::control::config::MotorGenome genome) {
  steer_genome_ = genome;
}

void SwerveModuleSubsystem::Setup() {
  using namespace funkit::control::config;
  using namespace pdcsu::units;

  if (!drive_genome_.has_value()) {
    throw std::runtime_error("Drive genome must be set before calling Setup()");
  }
  if (!steer_genome_.has_value()) {
    throw std::runtime_error("Steer genome must be set before calling Setup()");
  }

  drive_.Setup(drive_genome_.value(), drive_plant_);
  drive_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{5}, ms_t{5}, ms_t{20});
  drive_.SetPosition(meter_t{0});

  steer_.Setup(steer_genome_.value(), steer_plant_);
  steer_.EnableStatusFrames(
      {StatusFrame::kPositionFrame, StatusFrame::kVelocityFrame}, ms_t{20},
      ms_t{20}, ms_t{5}, ms_t{20});

  ZeroWithCANcoder();
}

SwerveModuleTarget SwerveModuleSubsystem::ZeroTarget() const {
  return SwerveModuleOLControlTarget{fps_t{0.0}, degree_t{0}};
}

bool SwerveModuleSubsystem::VerifyHardware() {
  bool ok = true;
  FUNKIT_VERIFY(drive_.VerifyConnected(), ok, "Could not verify drive motor");
  FUNKIT_VERIFY(steer_.VerifyConnected(), ok, "Could not verify steer motor");
  return ok;
}

void SwerveModuleSubsystem::SetCANCoderOffset() {
  units::degree_t position_wpi = cancoder_.GetAbsolutePosition().GetValue();
  SetCANCoderOffset(degree_t{position_wpi.to<double>()});
}
void SwerveModuleSubsystem::SetCANCoderOffset(degree_t offset) {
  SetPreferenceValue("cancoder_offset_", offset);
}

void SwerveModuleSubsystem::ZeroWithCANcoder() {
  if (frc::RobotBase::IsSimulation()) {
    steer_.SetPosition(radian_t{0});
    return;
  }

  constexpr int kMaxAttempts = 5;
  constexpr int kSleepTimeMs = 500;

  last_rezero = 0;

  for (int attempts = 1; attempts <= kMaxAttempts; ++attempts) {
    Log("CANCoder zero attempt {}/{}", attempts, kMaxAttempts);
    auto position = cancoder_.GetAbsolutePosition();

    auto offset_pdcsu =
        GetPreferenceValue_unit_type<degree_t>("cancoder_offset_");
    units::degree_t position_wpi = position.GetValue();
    degree_t position_zero =
        degree_t{-position_wpi.to<double>() + offset_pdcsu.value()};

    if (position.IsAllGood()) {
      radian_t position_zero_rad{position_zero};
      steer_.SetPosition(position_zero_rad);
      Log("Zeroed to {}!", position_zero.value());
      return;
    } else if (attempts == kMaxAttempts) {
      Error("Unable to zero normally after {} attempts - attempting anyways",
          kMaxAttempts);
      radian_t position_zero_rad{position_zero};
      steer_.SetPosition(position_zero_rad);
      Warn("Unreliably zeroed to {}!", position_zero.value());
      return;
    }

    Warn("Attempt to zero failed, sleeping {} ms...", kSleepTimeMs);

    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
  }
}

SwerveModuleReadings SwerveModuleSubsystem::ReadFromHardware() {
  SwerveModuleReadings readings;
  auto drive_vel_mps = drive_.GetVelocity<mps_t>();
  readings.vel = fps_t{drive_vel_mps.value() * 3.28084};
  auto drive_pos_m = drive_.GetPosition<meter_t>();
  readings.drive_pos = foot_t{drive_pos_m.value() * 3.28084};
  auto steer_pos_rad = steer_.GetPosition<radian_t>();
  readings.steer_pos = degree_t{steer_pos_rad};

  auto steer_vel_pdcsu = steer_.GetVelocity<radps_t>();
  nm_t pred_steer_load = nm_t{steer_load_factor_.value() *
                              readings.vel.value() * steer_vel_pdcsu.value()};

  steer_.SetLoad(pred_steer_load);

  Graph("readings/pred_steer_load", pred_steer_load);

  Graph("readings/drive_motor_vel", readings.vel);
  // Graph("readings/drive_motor_pos", readings.drive_pos);
  Graph("readings/steer_motor_pos", readings.steer_pos);

  units::degree_t cancoder_pos_wpi = cancoder_.GetAbsolutePosition().GetValue();
  Graph("readings/cancoder_pos", degree_t{cancoder_pos_wpi.to<double>()});

  return readings;
}

void SwerveModuleSubsystem::WriteToHardware(SwerveModuleTarget target) {
  // Graph("target/drive_target", target.drive);
  // Graph("target/steer_target", target.steer);
  auto [steer_dir, invert_drive] =
      calculateSteerPosition(target.steer, GetReadings().steer_pos);
  (void)invert_drive;

  Graph("target/steer_dir", steer_dir);

  degree_t steer_diff = target.steer - GetReadings().steer_pos;
  Graph("steer_error", steer_diff);
  double cosine_comp = std::cos(radian_t{steer_diff}.value());

  const auto& motor_specs =
      funkit::control::base::MotorSpecificationPresets::get(motor_types_);
  const amp_t stall_current{motor_specs.stall_current};
  const ohm_t winding_res = volt_t{12.0} / stall_current;

  double res_corr_factor =
      ((circuit_resistance_ + winding_res) / winding_res).value();
  const double avg_corr_factor =
      ((avg_resistance_ + winding_res) / winding_res).value();
  res_corr_factor /= avg_corr_factor;

  auto target_drive_val = target.drive.value();
  auto current_vel_val = GetReadings().vel.value();
  if (std::abs(target_drive_val) > std::abs(current_vel_val)) {
    target.drive =
        fps_t{(target_drive_val - current_vel_val) * res_corr_factor +
              current_vel_val};
  }
  Graph("res_corr_factor", res_corr_factor);
  Graph("avg_corr_factor", avg_corr_factor);

  // Graph("target/cosine_comp", cosine_comp);

  double drive_duty_cycle =
      cosine_comp * target.drive.value() / max_speed_.value();

  Graph("target/drive_dc", drive_duty_cycle);

  drive_.WriteDC(drive_duty_cycle);

  if (std::abs(target.drive.value()) > 0.04 || last_rezero < 50) {
    radian_t steer_dir_rad{steer_dir};
    steer_.WritePositionOnController(steer_dir_rad);
    last_rezero += 1;
  }
}

std::pair<degree_t, bool> SwerveModuleSubsystem::calculateSteerPosition(
    degree_t target_norm, degree_t current) {
  bool invert = false;

  degree_t target = target_norm;

  while ((target.value() - current.value()) > 90.0) {
    target = degree_t{target.value() - 180.0};
    invert = !invert;
  }
  while ((target.value() - current.value()) < -90.0) {
    target = degree_t{target.value() + 180.0};
    invert = !invert;
  }

  return {target, invert};
}

void SwerveModuleSubsystem::ModifyGenomes(
    funkit::control::config::MotorGenome genome_steer,
    funkit::control::config::MotorGenome genome_drive) {
  steer_.ModifyGenome(genome_steer);
  drive_.ModifyGenome(genome_drive);
}

}  // namespace funkit::robot::swerve