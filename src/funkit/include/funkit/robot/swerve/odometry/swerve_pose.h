#pragma once

#include "pdcsu_units.h"
#include "util/math/uvec.h"

namespace funkit::robot::swerve::odometry {

using Vector2D = pdcsu::util::math::uVec<pdcsu::units::inch_t, 2>;

struct SwervePose {
  Vector2D position;
  pdcsu::units::degree_t bearing;
  pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> velocity;

  SwervePose rotate(pdcsu::units::degree_t angle) const;
  SwervePose translate(Vector2D translation) const;

  SwervePose extrapolate(pdcsu::units::second_t time) const;

  SwervePose operator+(const SwervePose& other) const;
  SwervePose operator-(const SwervePose& other) const;
};

}  // namespace funkit::robot::swerve::odometry