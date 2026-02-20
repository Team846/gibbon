#include "calculators/ShootingCalculator.h"

#include <frc/DriverStation.h>

ShootingCalculatorOutputs ShootingCalculator::outputs_{
    0_deg_, 0_radps_, 60_deg_, 0_radps_, 0_fps_, false};
std::optional<funkit::base::Loggable> ShootingCalculator::loggable_opt;
pdcsu::util::math::Vector2D ShootingCalculator::target{0_in_, 0_in_};

const inch_t kDeltaHeight = 57.9_in_ - 24.9_in_;

const degree_t kShotAngleMax = 75_deg_;
const degree_t kShotAngleMin = 55_deg_;
const foot_t kShotMaxDist = 30_ft_;
const foot_t kPointblankDistance = 48_in_;

void ShootingCalculator::Setup() {
  if (loggable_opt.has_value()) { return; }
  loggable_opt.emplace(funkit::base::Loggable("ShootingCalculator"));
  loggable_opt->RegisterPreference("2ptvel/kPointBlank", 17_fps_);
  loggable_opt->RegisterPreference("2ptvel/kAdditive", 0.0);
  loggable_opt->RegisterPreference("swim/twistGain", 0.0);
  loggable_opt->RegisterPreference("swim/projectGain", 0.0);
  loggable_opt->RegisterPreference("swim/twistVelCompensation", 0.416);
}

degree_t ShootingCalculator::GetShotAngle(foot_t shot_distance) {
  return kShotAngleMax - (kShotAngleMax - kShotAngleMin) *
                             (shot_distance / kShotMaxDist).value();
}
radps_t ShootingCalculator::GetShotAngleVel(
    foot_t shot_distance, fps_t vel_in_dir) {
  return (kShotAngleMax - kShotAngleMin) *
         UnitDivision<inch_t, second_t>(vel_in_dir) / kShotMaxDist;
}

fps_t ShootingCalculator::GetBaseVelocity(
    degree_t shot_angle, foot_t shot_distance) {
  UnitCompound<fps_t, fps_t> physics_vel_sksq =
      16.0_fps2_ * shot_distance * shot_distance /
      (shot_distance * u_tan(shot_angle) - kDeltaHeight);
  return u_sqrt(physics_vel_sksq) / u_cos(shot_angle);
}

void ShootingCalculator::Calculate(const RobotContainer* container_) {
  if (!loggable_opt.has_value()) {
    throw std::runtime_error("ShootingCalculator not setup");
  }
  auto& loggable = loggable_opt.value();

  using namespace pdcsu::util::math;
  using Vel2D = uVec<pdcsu::units::fps_t, 2>;

  auto drivetrain_readings = container_->drivetrain_.GetReadings();

  /*
  The shooter may not be at the robot's center of rotation. The following code
  calculates the position of the shooter and the velocity at the shooter.
  */

  const Vector2D robot_to_shooter = Vector2D{0_in_, 0.9375_in_}.rotate(
      drivetrain_readings.estimated_pose.bearing, true);
  const Vector2D shooter_pos =
      drivetrain_readings.estimated_pose.position + robot_to_shooter;

  outputs_.start_traj = shooter_pos;

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
  fps_t vel_in_dir =
      u_sqrt(vel_at_shooter.magnitude() * vel_at_shooter.magnitude() -
             vel_perp * vel_perp);

  inch_t D_f_O = u_clamp(inch_t{delta_mag} - kPointblankDistance,
      kPointblankDistance, kShotMaxDist);

  loggable.Graph("D_f_O", D_f_O);
  outputs_.shot_angle = GetShotAngle(D_f_O);
  loggable.Graph("shot_angle", outputs_.shot_angle);
  loggable.Graph("shot_distance_ratio", (D_f_O / kShotMaxDist).value());
  outputs_.shot_angle_vel = GetShotAngleVel(D_f_O, vel_in_dir);

  /*
  2pt interpolation to get target shooting velocity.
  */
  auto shot_ptbvel = GetBaseVelocity(outputs_.shot_angle, kPointblankDistance);

  const second_t projectMultFac =
      loggable.GetPreferenceValue_double("swim/projectGain") * 1.0_s_ *
      1.0_ft_ / delta_mag;

  const Vector2D projectFwd =
      shooter_pos + Vector2D{vel_at_shooter[0] * projectMultFac,
                        vel_at_shooter[1] * projectMultFac};

  const foot_t fwdErrorMag =
      (target - projectFwd).magnitude() - kPointblankDistance;

  auto shot_vel = GetBaseVelocity(outputs_.shot_angle, fwdErrorMag);

  fps_t target_vel =
      loggable.GetPreferenceValue_unit_type<fps_t>("2ptvel/kPointBlank") +
      loggable.GetPreferenceValue_double("2ptvel/kAdditive") *
          (shot_vel - shot_ptbvel);

  outputs_.shooter_vel = target_vel;

  /*
  Calculate turret angles
  */

  degree_t aim_angle = delta.angle(true);

  const double robot_proj_vel_ratio =
      vel_perp.value() / outputs_.shooter_vel.value();
  const degree_t twist = u_asin(
      std::clamp(robot_proj_vel_ratio *
                     loggable.GetPreferenceValue_double("swim/twistGain"),
          -1.0, 1.0));

  loggable.Graph("swim/twist", twist);

  aim_angle += twist;

  outputs_.shooter_vel =
      outputs_.shooter_vel * 1.0 /
      u_cos(u_min(u_abs(twist * loggable.GetPreferenceValue_double(
                                    "swim/twistVelCompensation")),
          3.14159265358979323846_rad_ / 2.0));

  outputs_.aim_angle = aim_angle;

  auto cross_product =
      delta[0] * vel_at_shooter[1] - delta[1] * vel_at_shooter[0];
  auto distance_squared = delta_mag * delta_mag;

  outputs_.vel_aim_compensation =
      u_clamp(1_rad_ * (cross_product / distance_squared), -300_degps_,
          300_degps_);  // TODO: use max omega

  Vector2D proj_vel =
      projectFwd +
      Vector2D{1_in_, aim_angle + drivetrain_readings.pose.bearing, true}
          .resize(u_abs(vel_perp) * projectMultFac / 1_ft_ * 1_in_);
  foot_t d = (target - proj_vel).magnitude();
  loggable_opt->Graph("distance_vel", d);
  outputs_.is_valid = d >= kPointblankDistance && d <= 235.0_in_;

  outputs_.term_traj = SimulateTrajectory(container_) + outputs_.start_traj;
}

pdcsu::util::math::Vector2D ShootingCalculator::SimulateTrajectory(
    const RobotContainer* container_) {
  fps_t effective_launch_speed =
      container_->scorer_ss_.shooter.GetReadings().vel *
      0.92;  // Roughly 8% drop in speed when shot passes through

  fps_t shot_speed = effective_launch_speed *
                     u_cos(container_->scorer_ss_.hood.GetReadings().pos_);
  degree_t shot_dir = container_->drivetrain_.GetReadings().pose.bearing +
                      container_->scorer_ss_.turret.GetReadings().pos_;
  pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> shot_vel{
      shot_speed, shot_dir, true};

  pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> compl_vel =
      shot_vel + container_->drivetrain_.GetReadings().estimated_pose.velocity;

  fps_t vertical_vel = effective_launch_speed *
                       u_sin(container_->scorer_ss_.hood.GetReadings().pos_);
  second_t time_to_apex = vertical_vel / 32.2_fps2_;
  foot_t height_at_apex = vertical_vel / 2.0 * time_to_apex;

  const foot_t target_height =
      38.5_in_;  // Roughly the delta between target height and shooter

  second_t time_apex_to_target =
      u_sqrt(2 * u_max(0_in_, height_at_apex - target_height) / 32.2_fps2_);

  return {compl_vel[0] * u_max(0.25_s_, time_apex_to_target + time_to_apex),
      compl_vel[1] * u_max(0.25_s_, time_apex_to_target + time_to_apex)};
}