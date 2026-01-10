#pragma once

#include <array>

#include "funkit/math/calculator.h"
#include "pdcsu_units.h"
#include "util/math/uvec.h"

namespace funkit::robot::swerve::odometry {

using Vector2D = pdcsu::util::math::uVec<pdcsu::units::inch_t, 2>;

struct SwerveOdometryConstants {
  pdcsu::units::inch_t forward_wheelbase_dim;
  pdcsu::units::inch_t horizontal_wheelbase_dim;
};

struct SwerveOdometryInputs {
  pdcsu::units::degree_t bearing;
  std::array<pdcsu::units::degree_t, 4> steer_pos;
  pdcsu::util::math::uVec<pdcsu::units::inch_t, 4> drive_pos;
  double odom_ff_;
};

struct SwerveOdometryOutput {
  Vector2D position;
  pdcsu::units::degree_t odom_bearing;
};

class SwerveOdometryCalculator
    : public funkit::math::Calculator<SwerveOdometryInputs,
          SwerveOdometryOutput, SwerveOdometryConstants> {
public:
  SwerveOdometryCalculator();

  SwerveOdometryOutput calculate(SwerveOdometryInputs inputs) override;

  void SetPosition(Vector2D position) {
    position_offset_ = position - last_position_;
  }
  void SetOdomBearing(pdcsu::units::degree_t bearing) {
    odom_bearing_ = bearing;
  }

  pdcsu::units::degree_t GetOdomBearing() { return odom_bearing_; }

private:
  pdcsu::util::math::uVec<pdcsu::units::inch_t, 4> previous_module_positions_;

  Vector2D last_position_;

  pdcsu::units::degree_t odom_bearing_;

  Vector2D position_offset_{};
};

}  // namespace funkit::robot::swerve::odometry
