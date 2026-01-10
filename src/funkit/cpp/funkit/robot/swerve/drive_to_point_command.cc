#include "funkit/robot/swerve/drive_to_point_command.h"

#include <frc/RobotBase.h>

#include "funkit/wpilib/time.h"
#include "pdcsu_units.h"

namespace funkit::robot::swerve {

DriveToPointCommand::DriveToPointCommand(DrivetrainSubsystem* drivetrain,
    funkit::math::FieldPoint target, pdcsu::units::fps_t max_speed,
    pdcsu::units::fps2_t max_acceleration,
    pdcsu::units::fps2_t max_deceleration, DriveToPointFlags flags)
    : Loggable("DriveToPointCommand"),
      drivetrain_(drivetrain),
      max_speed_(max_speed),
      max_acceleration_(max_acceleration),
      max_deceleration_(max_deceleration),
      target_(target),
      flags_(flags) {
  SetName("DriveToPointCommand");
  AddRequirements({drivetrain_});
}

void DriveToPointCommand::Initialize() {
  Log("DriveToPointCommand initialized");

  auto readings = drivetrain_->GetReadings();
  start_point_ = readings.estimated_pose.position;
  start_time_ = funkit::wpilib::CurrentFPGATime();

  const auto [new_target_point, is_valid] = GetTargetPoint();
  if (is_valid) { target_ = new_target_point; }
  if (!(flags_ & kLockToPoint)) { target_.velocity = pdcsu::units::fps_t{0}; }

  auto initial_distance = (target_.point - start_point_).magnitude();
  auto initial_velocity = readings.estimated_pose.velocity.magnitude();
  estimated_time_ = EstimateCompletionTime(
      initial_distance, initial_velocity, target_.velocity);

  Log("Estimated completion time: {} seconds", estimated_time_.value());
}

void DriveToPointCommand::Execute() {
  DrivetrainReadings dt_readings{drivetrain_->GetReadings()};

  const auto [new_target_point, is_valid] = GetTargetPoint();

  const double kC = drivetrain_->GetPreferenceValue_double("drive_to_point/kC");
  const double kA = drivetrain_->GetPreferenceValue_double("drive_to_point/kA");
  const double kE = drivetrain_->GetPreferenceValue_double("drive_to_point/kE");

  DrivetrainTarget dt_target{{pdcsu::units::fps_t{0}, pdcsu::units::fps_t{0}},
      pdcsu::units::degps_t{0}};

  funkit::math::Vector2D delta_vec =
      target_.point - dt_readings.estimated_pose.position;

  pdcsu::units::degree_t heading_direction = delta_vec.angle(true);
  if (dt_readings.estimated_pose.velocity.magnitude().value() > 2.0)
    heading_direction = dt_readings.estimated_pose.velocity.angle(true);

  pdcsu::units::degree_t O = delta_vec.angle(true) - heading_direction;

  double O_rad = pdcsu::units::radian_t{O}.value();
  double delta_mag = delta_vec.magnitude().value();
  double max_dec_val = max_deceleration_.value();
  double vel_mag = dt_readings.estimated_pose.velocity.magnitude().value();

  double vel_dir_target_val =
      std::min(
          max_speed_.value(), (target_.velocity.value() +
                                  std::sqrt(delta_mag * kE * max_dec_val))) *
      std::cos(O_rad);

  double vel_lat_target_val =
      std::sin(O_rad) * std::sqrt(std::max(5.0, vel_mag) * 1.0) * kC;

  vel_lat_target_val = std::min(5.0, std::max(-5.0, vel_lat_target_val));

  double vel_dir_target_controlled_val =
      vel_dir_target_val + (vel_dir_target_val - vel_mag) * kA;

  vel_dir_target_controlled_val =
      std::min(15.0, std::max(-15.0, vel_dir_target_controlled_val));

  pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> vel_target{
      pdcsu::units::fps_t{vel_lat_target_val},
      pdcsu::units::fps_t{vel_dir_target_controlled_val}};
  vel_target = vel_target.rotate(heading_direction, true);

  dt_target.velocity = vel_target;
  dt_target.angular_velocity = drivetrain_->ApplyBearingPID(target_.bearing);
  dt_target.cut_excess_steering = true;
  dt_target.accel_clamp = max_acceleration_;

  if (delta_vec.magnitude().value() < 0.75) {
    dt_target.velocity = {pdcsu::units::fps_t{0}, pdcsu::units::fps_t{0}};
  }

  drivetrain_->SetTarget(dt_target);
}

void DriveToPointCommand::End(bool interrupted) {
  Log("DriveToPointCommand ended with interruption status {} at position "
      "x: {} "
      "and position y: {}",
      interrupted,
      drivetrain_->GetReadings().estimated_pose.position[0].value(),
      drivetrain_->GetReadings().estimated_pose.position[1].value());

  Log("The error at the end is x: {}, y:{}",
      (target_.point[0] - drivetrain_->GetReadings().estimated_pose.position[0])
          .value(),
      (target_.point[1] - drivetrain_->GetReadings().estimated_pose.position[1])
          .value());
  drivetrain_->SetTargetZero();
}

bool DriveToPointCommand::IsFinished() {
  if (flags_ & kLockToPoint) { return false; }

  auto drivetrain_readings = drivetrain_->GetReadings();
  auto current_point = drivetrain_readings.estimated_pose.position;

  if (flags_ & kRequireBearing) {
    auto bearing_error = funkit::math::CoterminalDifference(
        drivetrain_readings.estimated_pose.bearing, target_.bearing);
    auto bearing_threshold =
        drivetrain_->GetPreferenceValue_unit_type<pdcsu::units::degree_t>(
            "drive_to_point/bearing_threshold");
    if (pdcsu::units::u_abs(bearing_error) > bearing_threshold) {
      return false;
    }
  }

  auto distance_traveled = (current_point - start_point_).magnitude();
  auto total_distance = (target_.point - start_point_).magnitude();
  auto distance_remaining = (target_.point - current_point).magnitude();
  auto position_threshold =
      drivetrain_->GetPreferenceValue_unit_type<pdcsu::units::inch_t>(
          "drive_to_point/threshold");

  bool overshot = distance_traveled.value() >= total_distance.value();
  bool within_threshold =
      distance_remaining.value() < position_threshold.value();

  bool has_reached_target = overshot || within_threshold;

  if (flags_ & kNoTimeout) { return has_reached_target; }

  auto current_time = funkit::wpilib::CurrentFPGATime();
  auto elapsed_time = current_time - start_time_;
  constexpr double kTimeoutMultiplier = 1.5;
  constexpr second_t kAdditionalTimeout{0.75};
  auto timeout = estimated_time_ * kTimeoutMultiplier + kAdditionalTimeout;
  bool timed_out = elapsed_time > timeout;

  if (timed_out) {
    Warn("DriveToPointCommand timed out after {} seconds (estimated: {}, "
         "timeout: {})",
        elapsed_time.value(), estimated_time_.value(), timeout.value());
  }

  return has_reached_target || timed_out;
}

pdcsu::units::second_t DriveToPointCommand::EstimateCompletionTime(
    pdcsu::units::inch_t distance, pdcsu::units::fps_t initial_velocity,
    pdcsu::units::fps_t final_velocity) const {
  using namespace pdcsu::units;

  foot_t distance_ft{distance.value() / 12.0};

  fps_t v0 = initial_velocity;
  fps_t vf = final_velocity;
  fps_t vmax = max_speed_;
  fps2_t accel = max_acceleration_;
  fps2_t decel = max_deceleration_;

  fps_t accel_speed_gain = vmax - v0;
  fps_t decel_speed_loss = vmax - vf;

  second_t t_accel{0};
  foot_t d_accel{0};
  if (accel_speed_gain.value() > 0) {
    t_accel = second_t{accel_speed_gain.value() / accel.value()};
    d_accel = foot_t{v0.value() * t_accel.value() +
                     0.5 * accel.value() * t_accel.value() * t_accel.value()};
  }

  second_t t_decel{0};
  foot_t d_decel{0};
  if (decel_speed_loss.value() > 0) {
    t_decel = second_t{decel_speed_loss.value() / decel.value()};
    d_decel = foot_t{vmax.value() * t_decel.value() -
                     0.5 * decel.value() * t_decel.value() * t_decel.value()};
  }

  foot_t d_cruise =
      foot_t{distance_ft.value() - d_accel.value() - d_decel.value()};
  second_t t_cruise{0};

  if (d_cruise.value() > 0) {
    t_cruise = second_t{d_cruise.value() / vmax.value()};
  } else {
    foot_t adjusted_distance = distance_ft;
    fps_t peak_velocity = fps_t{std::sqrt(
        (2.0 * accel.value() * decel.value() * adjusted_distance.value() +
            decel.value() * v0.value() * v0.value() +
            accel.value() * vf.value() * vf.value()) /
        (accel.value() + decel.value()))};

    if (peak_velocity.value() < vmax.value()) {
      t_accel = second_t{(peak_velocity.value() - v0.value()) / accel.value()};
      t_decel = second_t{(peak_velocity.value() - vf.value()) / decel.value()};
    }
  }

  second_t total_time = t_accel + t_cruise + t_decel;

  constexpr double kMinTime = 0.5;
  if (total_time.value() < kMinTime) { total_time = second_t{kMinTime}; }

  return total_time;
}

}  // namespace funkit::robot::swerve