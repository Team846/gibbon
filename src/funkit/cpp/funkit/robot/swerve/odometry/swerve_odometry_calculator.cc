#include "funkit/robot/swerve/odometry/swerve_odometry_calculator.h"

#include <algorithm>
#include <cmath>

#include "pdcsu_units.h"
#include "util/math/uvec.h"

using Vec2D = funkit::robot::swerve::odometry::Vector2D;
using inch_sq_t =
    pdcsu::units::UnitCompound<pdcsu::units::inch_t, pdcsu::units::inch_t>;

#define w_h \
  pdcsu::units::inch_t { constants_.horizontal_wheelbase_dim / 2.0 }
#define w_f \
  pdcsu::units::inch_t { constants_.forward_wheelbase_dim / 2.0 }

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

  constexpr int kNumWheels = 4;
  std::array<inch_t, kNumWheels> wx;
  std::array<inch_t, kNumWheels> wy;
  std::array<inch_t, kNumWheels> r_x;
  std::array<inch_t, kNumWheels> r_y;
  for (int i = 0; i < kNumWheels; i++) {
    wx[i] = wheel_vecs[i][0];
    wy[i] = wheel_vecs[i][1];
    r_x[i] = wheel_positions[i][0];
    r_y[i] = wheel_positions[i][1];
  }

  std::array<pdcsu::units::scalar_t, kNumWheels> w;
  for (int i = 0; i < kNumWheels; i++) {
    w[i] = pdcsu::units::scalar_t{1.0};
  }

  constexpr int kMaxIterations = 3;
  constexpr pdcsu::units::inch_t kHuberScale{0.1};
  inch_t dx{0};
  inch_t dy{0};
  radian_t dtheta{0};

  for (int iter = 0; iter < kMaxIterations; iter++) {
    scalar_t W_d{0};
    inch_t W_x{0};
    inch_t W_y{0};
    inch_sq_t W_r2{0};
    inch_t S_wx{0};
    inch_t S_wy{0};
    inch_sq_t S_theta{0};
    for (int i = 0; i < kNumWheels; i++) {
      scalar_t wi = w[i];
      W_d += wi;
      W_x += wi * r_x[i];
      W_y += wi * r_y[i];
      W_r2 += wi * (r_x[i] * r_x[i] + r_y[i] * r_y[i]);
      S_wx += wi * wx[i];
      S_wy += wi * wy[i];
      S_theta += wi * (-r_y[i] * wx[i] + r_x[i] * wy[i]);
    }
    if (W_d.value() < 1e-12) {
      W_d = 1e-12_u_;
    }
    inch_sq_t denom =
        W_r2 - (W_x * W_x + W_y * W_y) / W_d;
    if (pdcsu::units::u_abs(denom).value() < 1e-12) {
      denom = (denom >= 0_in_ * 1_in_) ? 1e-12_in_ * 1_in_ : -1e-12_in_ * 1_in_;
    }
    scalar_t dtheta_ratio =
        (S_theta + (W_y * S_wx - W_x * S_wy) / W_d) / denom;
    dtheta = radian_t{dtheta_ratio.value()};
    dx = (S_wx + W_y * dtheta) / W_d;
    dy = (S_wy - W_x * dtheta) / W_d;

    std::array<pdcsu::units::inch_t, kNumWheels> res;
    for (int i = 0; i < kNumWheels; i++) {
      inch_t wx_pred = dx - dtheta * r_y[i];
      inch_t wy_pred = dy + dtheta * r_x[i];
      inch_t ex = wx[i] - wx_pred;
      inch_t ey = wy[i] - wy_pred;
      res[i] = u_sqrt(ex * ex + ey * ey);
    }
    std::array<inch_t, kNumWheels> res_sorted = res;
    std::sort(res_sorted.begin(), res_sorted.end(),
              [](inch_t a, inch_t b) {
                return a < b;
              });
    inch_t median_res =
        (res_sorted[1] + res_sorted[2]) * 0.5;
    inch_t scale = median_res + kHuberScale;
    if (scale < 1e-9_in_) {
      scale = 1e-9_in_;
    }
    for (int i = 0; i < kNumWheels; i++) {
      scalar_t u = res[i] / scale;
      w[i] = (u <= 1.0_u_) ? 1.0_u_ : 1.0_u_ / u;
    }
  }

  Vec2D displacement;

  constexpr pdcsu::units::radian_t EPSILON_dtheta{1e-6};

  if (pdcsu::units::u_abs(dtheta) > EPSILON_dtheta) {
    double dtheta_val = dtheta.value();
    double sin_term = pdcsu::units::u_sin(dtheta) / dtheta_val;
    double cos_term = (1.0 - pdcsu::units::u_cos(dtheta)) / dtheta_val;

    displacement = Vec2D{dx * sin_term - dy * cos_term,
                         dx * cos_term + dy * sin_term};
  } else {
    displacement = Vec2D{dx, dy};
  }

  displacement *= inputs.odom_ff_;

  last_position_ += displacement.rotate(inputs.bearing - dtheta);

  odom_bearing_ += dtheta;

  return {last_position_ + position_offset_, odom_bearing_};
}

}  // namespace funkit::robot::swerve::odometry