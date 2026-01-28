#include "calculators/ShootingCalculator.h"

ShootingCalculatorOutputs ShootingCalculator::outputs_{
    0_deg_, 0_radps_, 0_fps_, false};
std::optional<funkit::base::Loggable> ShootingCalculator::loggable_opt;

void ShootingCalculator::Setup() {
  if (loggable_opt.has_value()) { return; }
  loggable_opt.emplace(funkit::base::Loggable("ShootingCalculator"));
  loggable_opt->RegisterPreference("2ptvel/kPointBlank", 17_fps_);
  loggable_opt->RegisterPreference("2ptvel/kAdditive", 0.0);
  loggable_opt->RegisterPreference("swim/twistGain", 0.0);
  loggable_opt->RegisterPreference("swim/projectGain", 0.0);
}

void ShootingCalculator::Calculate(const RobotContainer* container_) {
  if (!loggable_opt.has_value()) {
    throw std::runtime_error("ShootingCalculator not setup");
  }
  auto& loggable = loggable_opt.value();

  using namespace pdcsu::util::math;

  using Vel2D = uVec<pdcsu::units::fps_t, 2>;

  const Vector2D target{158.845_in_, 182.11_in_};  // TODO: Blue side flipping
  const inch_t pointblank_distance = 48_in_;

  auto drivetrain_readings = container_->drivetrain_.GetReadings();

  /*
  The shooter may not be at the robot's center of rotation. The following code
  calculates the position of the shooter and the velocity at the shooter.
  */

  const Vector2D robot_to_shooter = Vector2D{0_in_, -6_in_}.rotate(
      drivetrain_readings.estimated_pose.bearing, true);
  const Vector2D shooter_pos =
      drivetrain_readings.estimated_pose.position + robot_to_shooter;

  const auto tangential_speed =
      robot_to_shooter.magnitude() *
      pdcsu::units::radps_t{drivetrain_readings.yaw_rate} / 1_rad_;
  const Vel2D vel_at_shooter =
      drivetrain_readings.estimated_pose.velocity +
      Vel2D{tangential_speed, robot_to_shooter.angle(true) + 90_deg_,
          true};  // TODO: verify signs

  const Vector2D delta = target - shooter_pos;
  const foot_t delta_mag = delta.magnitude();

  /*
  Find velocity towards the target and perpendicular to it (CCW is positive).
  */
  fps_t vel_perp =
      delta.rotate(90_deg_).dot(vel_at_shooter) / delta.magnitude();
  //   fps_t vel_in_dir =
  //       u_sqrt(vel_at_shooter.magnitude() * vel_at_shooter.magnitude() -
  //              vel_perp * vel_perp);

  /*
  2pt interpolation to get target shooting velocity.
  */
  const fps_t kPtBlank =
      loggable.GetPreferenceValue_unit_type<fps_t>("2ptvel/kPointBlank");
  const double kAdditive =
      loggable.GetPreferenceValue_double("2ptvel/kAdditive");

  const second_t projectMultFac =
      loggable.GetPreferenceValue_double("swim/projectGain") * 1.0_s_ *
      1.0_ft_ / delta_mag;

  const Vector2D projectFwd =
      shooter_pos + Vector2D{vel_at_shooter[0] * projectMultFac,
                        vel_at_shooter[1] * projectMultFac};

  const foot_t fwdErrorMag =
      (target - projectFwd).magnitude() - pointblank_distance;

  const fps_t shooter_vel = kPtBlank + kAdditive * fwdErrorMag / 1.0_s_ -
                            0.01 * kAdditive * kAdditive * fwdErrorMag *
                                fwdErrorMag / 1.0_s_ / 1.0_ft_;

  outputs_.shooter_vel = shooter_vel;

  /*
  Calculate drivetrain angles
  */

  degree_t aim_angle =
      drivetrain_readings.estimated_pose.position.angleAimTowards(
          target, true);  // <-- Use delta.angle() with turret

  const double robot_proj_vel_ratio = vel_perp.value() / shooter_vel.value();
  const degree_t twist = u_asin(
      std::clamp(robot_proj_vel_ratio *
                     loggable.GetPreferenceValue_double("swim/twistGain"),
          -1.0, 1.0));

  loggable.Graph("swim/twist", twist);

  aim_angle += twist;

  outputs_.aim_angle = aim_angle + 180_deg_;

  auto cross_product =
      delta[0] * vel_at_shooter[1] - delta[1] * vel_at_shooter[0];
  auto distance_squared = delta_mag * delta_mag;

  outputs_.vel_aim_compensation =
      u_clamp(-1_rad_ * (cross_product / distance_squared), -300_degps_,
          300_degps_);  // <-- TODO: use max omega

  /* Determine if the shot is valid */
  outputs_.is_valid =
      delta_mag >= 36_in_ && delta_mag <= 300_in_;  // TODO: verify distances
}