#pragma once

#include "pdcsu_units.h"
#include "util/math/uvec.h"

namespace funkit::robot::swerve::odometry {

using Vector2D = pdcsu::util::math::uVec<pdcsu::units::inch_t, 2>;

/**
 * SwervePose
 * 
 * A struct that holds information for 2D robot pose. Contains methods to manually adjust SwervePose information.
 */
struct SwervePose {
  Vector2D position; 
  pdcsu::units::degree_t bearing;
  pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> velocity;

  /**
   * rotate()
   * 
   * Manually adjusts the SwervePose values based on additional rotation. Rotates position and velocity by angle, and adds angle to bearing
   * @param angle - the additional rotation to add
   * @return a SwervePose with updated information
   */
  SwervePose rotate(pdcsu::units::degree_t angle) const;
  /**
   * translate()
   * 
   * Manually adjusts the SwervePose position based on additional translational movement
   * @param translation - the additional translational movement
   * @return a SwervePose with updated information
   */
  SwervePose translate(Vector2D translation) const;

  /**
   * extrapolate()
   * 
   * Extrapolates future location of SwervePose by using simple calculations involving only velocity.
   * When used in small time intervals, provides fairly accurate predictions since acceleration cannot drastically change velocity.
   * @param time - the duration forward where we wish to extrapolate
   * @return a SwervePose with an extrapolated position
   */
  SwervePose extrapolate(pdcsu::units::second_t time) const;

  /**
   * Operator+ overload function for SwervePose
   */
  SwervePose operator+(const SwervePose& other) const;
  /**
   * Operator- overload function for SwervePose
   */
  SwervePose operator-(const SwervePose& other) const;
};

}  // namespace funkit::robot::swerve::odometry