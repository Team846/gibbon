#pragma once

#include <cmath>

#include "pdcsu_units.h"
#include "util/math/uvec.h"

namespace funkit::math {

using Vector2D = pdcsu::util::math::uVec<pdcsu::units::inch_t, 2>;

/**
 * FieldPoint
 *
 * A struct used to provide information and functionality for a field location.
 */
struct FieldPoint {
  Vector2D point;
  pdcsu::units::degree_t bearing;

  pdcsu::units::fps_t velocity;

  // Returns the mirrored location onto the opposite end of the field if
  // applicable
  FieldPoint mirror(bool shouldMirror) const {
    if (shouldMirror) {
      return FieldPoint{{field_size_x - point[0], field_size_y - point[1]},
          pdcsu::units::degree_t{180} + bearing, velocity};
    }
    return FieldPoint{{point[0], point[1]}, bearing, velocity};
  }

  // Returns a field point with a mirrored y position.
  FieldPoint mirrorOnlyY(bool shouldMirror) const {
    if (shouldMirror) {
      return FieldPoint{{point[0], field_size_y - point[1]},
          pdcsu::units::degree_t{180} - bearing, velocity};
    }
    return FieldPoint{{point[0], point[1]}, bearing, velocity};
  }

  // Returns a field point with a mirrored x position.
  FieldPoint mirrorOnlyX(bool shouldMirror) const {
    if (shouldMirror) {
      return FieldPoint{
          {field_size_x - point[0], point[1]}, -bearing, velocity};
    }
    return FieldPoint{{point[0], point[1]}, bearing, velocity};
  }

  // Returns a FieldPoint with the direction of robot bearing flipped. Keeps
  // everything else the same.
  FieldPoint flipDirection() const {
    return FieldPoint{
        {point[0], point[1]}, bearing + pdcsu::units::degree_t{180}, velocity};
  }

  // Welded
  static constexpr pdcsu::units::inch_t field_size_y =
      pdcsu::units::inch_t{651.22};
  static constexpr pdcsu::units::inch_t field_size_x =
      pdcsu::units::inch_t{317.69};
};

}  // namespace funkit::math