#include "calculators/ForceFieldCalculator.h"

#include <algorithm>
#include <cmath>

std::vector<ForceFieldCalculator::ForceVector>
    ForceFieldCalculator::kHubForceVectors_ =
        ForceFieldCalculator::CreateHubForceVectors();

std::optional<funkit::base::Loggable> ForceFieldCalculator::loggable_opt_;

void ForceFieldCalculator::Setup() {
  if (loggable_opt_.has_value()) { return; }
  loggable_opt_.emplace(funkit::base::Loggable("ForceFieldCalculator"));

  loggable_opt_->RegisterPreference("wall/enabled", true);
  loggable_opt_->RegisterPreference("wall/safety_margin", 6_in_);
  loggable_opt_->RegisterPreference("wall/max_accel", 20.0);
  loggable_opt_->RegisterPreference("wall/activation_distance", 180.0_in_);
  loggable_opt_->RegisterPreference("wall/min_velocity", 2.0);

  loggable_opt_->RegisterPreference("hub/enabled", true);
  loggable_opt_->RegisterPreference("hub/k", 2.5);
  loggable_opt_->RegisterPreference("hub/activation_distance", 72.0_in_);
}

std::vector<ForceFieldCalculator::ForceVector>
ForceFieldCalculator::CreateHubForceVectors() {
  std::vector<ForceVector> vectors;

  const int vectors_per_side = 12;

  for (int i = 0; i < vectors_per_side; ++i) {
    double t = i / (vectors_per_side - 1.0);
    double y = kRedHubLeft + (kRedHubRight - kRedHubLeft) * t;
    double x = kRedHubFront + (kRedHubBack - kRedHubFront) * t;

    vectors.push_back({kRedHubFront, y, -1.0, 0.0});
    vectors.push_back({kRedHubBack, y, 1.0, 0.0});
    vectors.push_back({x, kRedHubLeft, 0.0, -1.0});
    vectors.push_back({x, kRedHubRight, 0.0, 1.0});

    double blue_y = kBlueHubLeft + (kBlueHubRight - kBlueHubLeft) * t;
    double blue_x = kBlueHubFront + (kBlueHubBack - kBlueHubFront) * t;

    vectors.push_back({kBlueHubFront, blue_y, -1.0, 0.0});
    vectors.push_back({kBlueHubBack, blue_y, 1.0, 0.0});
    vectors.push_back({blue_x, kBlueHubLeft, 0.0, -1.0});
    vectors.push_back({blue_x, kBlueHubRight, 0.0, 1.0});
  }

  return vectors;
}

pdcsu::util::math::uVec<fps_t, 2> ForceFieldCalculator::ApplyForceFields(
    pdcsu::util::math::uVec<fps_t, 2> velocity,
    const RobotContainer* container_) {
  if (!loggable_opt_.has_value()) { return velocity; }

  velocity = ApplyWallForceField(velocity, container_);
  velocity = ApplyHubForceField(velocity, container_);

  return velocity;
}

pdcsu::util::math::uVec<fps_t, 2> ForceFieldCalculator::ApplyWallForceField(
    pdcsu::util::math::uVec<fps_t, 2> velocity,
    const RobotContainer* container_) {
  auto& loggable = loggable_opt_.value();

  inch_t robot_x =
      container_->drivetrain_.GetReadings().estimated_pose.position[0];
  inch_t robot_y =
      container_->drivetrain_.GetReadings().estimated_pose.position[1];

  bool wall_enabled = loggable.GetPreferenceValue_bool("wall/enabled");
  if (!wall_enabled) { return velocity; }

  inch_t safety_margin =
      loggable.GetPreferenceValue_unit_type<inch_t>("wall/safety_margin");
  double max_accel = loggable.GetPreferenceValue_double("wall/max_accel");
  inch_t activation_distance =
      loggable.GetPreferenceValue_unit_type<inch_t>("wall/activation_distance");
  double min_velocity = loggable.GetPreferenceValue_double("wall/min_velocity");

  inch_t kFieldSizeX = funkit::math::FieldPoint::field_size_x;
  inch_t kFieldSizeY = funkit::math::FieldPoint::field_size_y;

  inch_t robot_half_x = robot_constants::base::wheelbase_x / 2.0;
  inch_t robot_half_y = robot_constants::base::wheelbase_y / 2.0;

  double d_left = (robot_x - robot_half_x - safety_margin).value();
  double d_right =
      (kFieldSizeX - robot_x - robot_half_x - safety_margin).value();
  double d_bottom = (robot_y - robot_half_y - safety_margin).value();
  double d_top = (kFieldSizeY - robot_y - robot_half_y - safety_margin).value();

  double vel_x = velocity[0].value();
  double vel_y = velocity[1].value();

  double result_vel_x = vel_x;
  double result_vel_y = vel_y;

  auto apply_wall = [&](double vel, double dist, bool moving_toward,
                        double& result) {
    if (!moving_toward || dist >= activation_distance.value()) return;

    double dist_ft = dist / 12.0;
    double max_safe_vel = std::sqrt(2.0 * max_accel * dist_ft);
    double smooth = 1.0 - (dist / activation_distance.value());
    double target_vel = std::max(min_velocity, max_safe_vel);

    if (vel < 0) target_vel = -target_vel;

    if (std::abs(vel) > target_vel) {
      result = vel + smooth * (target_vel - vel);
      result = (vel < 0) ? std::max(result, target_vel)
                         : std::min(result, target_vel);
    }
  };

  apply_wall(vel_x, d_left, vel_x < 0, result_vel_x);
  apply_wall(vel_x, d_right, vel_x > 0, result_vel_x);
  apply_wall(vel_y, d_bottom, vel_y < 0, result_vel_y);
  apply_wall(vel_y, d_top, vel_y > 0, result_vel_y);

  double closest_wall_dist = std::min({d_left, d_right, d_bottom, d_top});
  if (closest_wall_dist < activation_distance.value()) {
    double damping =
        0.5 + 2 * (closest_wall_dist / activation_distance.value());

    if (d_left < activation_distance.value() ||
        d_right < activation_distance.value()) {
      result_vel_y *= damping;
    }
    if (d_bottom < activation_distance.value() ||
        d_top < activation_distance.value()) {
      result_vel_x *= damping;
    }
  }

  return {fps_t{result_vel_x}, fps_t{result_vel_y}};
}

pdcsu::util::math::uVec<fps_t, 2> ForceFieldCalculator::ApplyHubForceField(
    pdcsu::util::math::uVec<fps_t, 2> velocity,
    const RobotContainer* container_) {
  auto& loggable = loggable_opt_.value();

  inch_t robot_x =
      container_->drivetrain_.GetReadings().estimated_pose.position[0];
  inch_t robot_y =
      container_->drivetrain_.GetReadings().estimated_pose.position[1];

  bool hub_enabled = loggable.GetPreferenceValue_bool("hub/enabled");
  if (!hub_enabled) { return velocity; }

  double hub_k = loggable.GetPreferenceValue_double("hub/k");
  inch_t hub_activation_distance =
      loggable.GetPreferenceValue_unit_type<inch_t>("hub/activation_distance");

  double robot_half_x_in = robot_constants::base::wheelbase_x.value() / 2.0;
  double robot_half_y_in = robot_constants::base::wheelbase_y.value() / 2.0;

  double total_decel_x = 0.0;
  double total_decel_y = 0.0;

  for (const auto& fv : kHubForceVectors_) {
    double dx = robot_x.value() - fv.pos_x;
    double dy = robot_y.value() - fv.pos_y;

    double edge_offset_x = (dx > 0) ? -robot_half_x_in : robot_half_x_in;
    double edge_offset_y = (dy > 0) ? -robot_half_y_in : robot_half_y_in;

    double edge_dx = dx + edge_offset_x;
    double edge_dy = dy + edge_offset_y;

    double r_squared = edge_dx * edge_dx + edge_dy * edge_dy;
    double r = std::sqrt(r_squared);

    double distance_factor = 1.0 - (r / hub_activation_distance.value());
    double force_magnitude = (hub_k / r_squared) * distance_factor;

    double vel_x_fps = velocity[0].value();
    double vel_y_fps = velocity[1].value();

    double vel_along_force = vel_x_fps * fv.dir_x + vel_y_fps * fv.dir_y;

    if (vel_along_force < 0) {
      double decel_magnitude = force_magnitude * std::abs(vel_along_force);

      total_decel_x += decel_magnitude * fv.dir_x;
      total_decel_y += decel_magnitude * fv.dir_y;
    }
  }

  loggable.Graph("hub/total_decel_x", total_decel_x);
  loggable.Graph("hub/total_decel_y", total_decel_y);

  double result_vel_x = velocity[0].value() + total_decel_x;
  double result_vel_y = velocity[1].value() + total_decel_y;

  return {fps_t{result_vel_x}, fps_t{result_vel_y}};
}
