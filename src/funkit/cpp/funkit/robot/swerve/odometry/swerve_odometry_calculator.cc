#include "funkit/robot/swerve/odometry/swerve_odometry_calculator.h"

#include "pdcsu_units.h"
#include "util/math/uvec.h"

using Vec2D = funkit::robot::swerve::odometry::Vector2D;

#define w_h \
  pdcsu::units::inch_t { constants_.horizontal_wheelbase_dim.value() / 2.0 }
#define w_f \
  pdcsu::units::inch_t { constants_.forward_wheelbase_dim.value() / 2.0 }

namespace funkit::robot::swerve::odometry {

SwerveOdometryCalculator::SwerveOdometryCalculator() {}

SwerveOdometryOutput SwerveOdometryCalculator::calculate(
    SwerveOdometryInputs inputs) {
  std::array<Vec2D, 4> wheel_positions{
      Vec2D{w_h, w_f}, Vec2D{-w_h, w_f}, Vec2D{-w_h, -w_f}, Vec2D{w_h, -w_f}};

  auto module_diffs = inputs.drive_pos - previous_module_positions_;
  previous_module_positions_ = inputs.drive_pos;

  std::array<Vec2D, 4> wheel_vecs;
  for (int i = 0; i < 4; i++) {
    wheel_vecs[i] = Vec2D{module_diffs[i], inputs.steer_pos[i], true};
  }

  int min_idx = 0;
  pdcsu::units::inch_t min_mag = wheel_vecs[0].magnitude();
  for (int i = 1; i < 4; i++) {
    pdcsu::units::inch_t mag = wheel_vecs[i].magnitude();
    if (mag < min_mag) {
      min_mag = mag;
      min_idx = i;
    }
  }

  Vec2D min_trans_vec = wheel_vecs[min_idx];
  Vec2D min_wheel_pos = wheel_positions[min_idx];

  pdcsu::units::inch_t dx = min_trans_vec[0];
  pdcsu::units::inch_t dy = min_trans_vec[1];

  double md_x = min_trans_vec[0].value();
  double md_y = min_trans_vec[1].value();
  double r_x = min_wheel_pos[0].value();
  double r_y = min_wheel_pos[1].value();
  double rr = r_x * r_x + r_y * r_y;

  pdcsu::units::radian_t dtheta{(md_x * r_y - md_y * r_x) / rr};

  Vec2D displacement;

  constexpr pdcsu::units::radian_t EPSILON_dtheta{1e-6};

  if (pdcsu::units::u_abs(dtheta).value() > EPSILON_dtheta.value()) {
    double dtheta_val = dtheta.value();

    double sin_term = pdcsu::units::u_sin(dtheta) / dtheta_val;
    double cos_term = (1 - pdcsu::units::u_cos(dtheta)) / dtheta_val;

    displacement = Vec2D{
        pdcsu::units::inch_t{sin_term * dx.value() - cos_term * dy.value()},
        pdcsu::units::inch_t{cos_term * dx.value() + sin_term * dy.value()}};
  } else {
    displacement = Vec2D{dx, dy};
  }

  displacement *= inputs.odom_ff_;

  last_position_ += displacement.rotate(inputs.bearing - dtheta);

  odom_bearing_ += dtheta;

  return {last_position_ + position_offset_, odom_bearing_};
}

}  // namespace funkit::robot::swerve::odometry