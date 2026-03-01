#include "calculators/ShootingCalculator.h"

#include <frc/DriverStation.h>

ShootingCalculatorOutputs ShootingCalculator::outputs_{
    0_deg_, 0_radps_, 60_deg_, 0_radps_, 0_fps_, false};
std::optional<funkit::base::Loggable> ShootingCalculator::loggable_opt;
pdcsu::util::math::Vector2D ShootingCalculator::target{0_in_, 0_in_};

const inch_t kDeltaHeight = 57.9_in_ - 24.9_in_;

const degree_t kShotAngleMax = 75_deg_;
const degree_t kShotAngleMin = 50_deg_;
const foot_t kShotMaxDist = 24_ft_;
const foot_t kPointblankDistance = 41.925_in_;

const foot_t fullEffortDistance = 235.0_in_;

void ShootingCalculator::Setup() {
  if (loggable_opt.has_value()) { return; }
  loggable_opt.emplace(funkit::base::Loggable("ShootingCalculator"));
  loggable_opt->RegisterPreference("2ptvel/kPointBlank", 23_fps_);
  loggable_opt->RegisterPreference("2ptvel/kAdditive", 3.68);
  loggable_opt->RegisterPreference("swim/twistGain", 0.0);
  loggable_opt->RegisterPreference("swim/tofGain", 0.0);
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
  const auto denom = shot_distance * u_tan(shot_angle) - kDeltaHeight;
  if (denom <= 0.02_in_) { return 0_fps_; }
  UnitCompound<fps_t, fps_t> physics_vel_sksq =
      16.0_fps2_ * shot_distance * shot_distance / denom;
  return u_sqrt(physics_vel_sksq) / u_cos(shot_angle);
}

void ShootingCalculator::Calculate(
    const RobotContainer* container_, bool effort_when_invald) {
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

  const Vel2D vel_at_shooter =
      drivetrain_readings.estimated_pose
          .velocity;  // Robot-to-shooter is small enough, that term may be
                      // considered zero

  const Vector2D odelta = target - shooter_pos;
  Vector2D delta = odelta;

  if (odelta.magnitude() < 0.03_in_) { return; }

  fps_t vel_in_dir = odelta.dot(vel_at_shooter) / odelta.magnitude();
  fps_t vel_perp = odelta.rotate(90_deg_, true).dot(vel_at_shooter) / odelta.magnitude();
  foot_t delta_mag = odelta.magnitude();

  second_t tof = odelta.magnitude() / 100_in_ *
                 0.375_s_;  // Initial guess (likely lower than actual)
                            //   degree_t shot_angle = 75_deg_;

  // Recursive iteration to find TOF
  for (int i = 0; i < 7; i++) {
    auto delta2 =
        odelta - Vector2D{vel_at_shooter[0] * tof, vel_at_shooter[1] * tof};
    auto delta_mag2 = delta2.magnitude();

    if (delta_mag2 < kPointblankDistance) {
      return;
    }  // Works when approaching TOF from below

    /*
    2pt interpolation to get target shooting velocity.
    */

    auto shot_angle = GetShotAngle(delta_mag - kPointblankDistance);
    auto shot_vel = GetBaseVelocity(shot_angle, delta_mag);

    tof = delta_mag / (shot_vel * u_cos(shot_angle));
  }

  delta = target - shooter_pos -
          Vector2D{vel_at_shooter[0] * tof *
                       loggable.GetPreferenceValue_double("swim/tofGain"),
              vel_at_shooter[1] * tof *
                  loggable.GetPreferenceValue_double("swim/tofGain")}; /*loggable.GetPreferenceValue_double("swim/tofGain")
* delta
+ (1.0 - loggable.GetPreferenceValue_double("swim/tofGain")) * odelta;*/
  delta_mag = delta.magnitude();

  /* Apply shooter and hood targets */
  outputs_.shot_angle = GetShotAngle(delta_mag - kPointblankDistance);
  outputs_.shot_angle_vel = GetShotAngleVel(delta_mag, vel_in_dir);
  auto shot_ptbvel = GetBaseVelocity(outputs_.shot_angle, kPointblankDistance);

  loggable.Graph("shot_angle", outputs_.shot_angle);
  loggable.Graph("vel_perp", vel_perp);
  auto shot_vel = GetBaseVelocity(outputs_.shot_angle, delta_mag);

  outputs_.shooter_vel =
      loggable.GetPreferenceValue_unit_type<fps_t>("2ptvel/kPointBlank") +
      loggable.GetPreferenceValue_double("2ptvel/kAdditive") *
          (shot_vel - shot_ptbvel);

  /* Calculate and apply turret targets */
  outputs_.aim_angle = odelta.angle(true) + u_asin(std::clamp(loggable.GetPreferenceValue_double("swim/twistGain") * (vel_perp / outputs_.shooter_vel).value(), -1.0, 1.0));

  auto cross_product =
      delta[0] * vel_at_shooter[1] - delta[1] * vel_at_shooter[0];
  auto distance_squared = delta_mag * delta_mag;

  outputs_.vel_aim_compensation = u_clamp(
      1_rad_ * (cross_product / distance_squared), -300_degps_, 300_degps_);

  /* Determine shot validity and whether to apply full effort */
  outputs_.is_valid =
      delta_mag >= kPointblankDistance && delta_mag <= fullEffortDistance;

  if (delta_mag > fullEffortDistance) {
    if (effort_when_invald) {
      outputs_.shooter_vel = 50_fps_;
    } else {
      outputs_.shooter_vel = 0_fps_;
    }
  }

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