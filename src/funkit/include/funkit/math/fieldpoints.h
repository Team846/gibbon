#pragma once

#include <cmath>

#include "pdcsu_units.h"
#include "util/math/uvec.h"

namespace funkit::math {

using Vector2D = pdcsu::util::math::uVec<pdcsu::units::inch_t, 2>;

struct FieldPoint {
  Vector2D point;
  pdcsu::units::degree_t bearing;

  pdcsu::units::fps_t velocity;

  FieldPoint mirror(bool shouldMirror) const {
    if (shouldMirror) {
      return FieldPoint{{field_size_x - point[0], field_size_y - point[1]},
          pdcsu::units::degree_t{180} + bearing, velocity};
    }
    return FieldPoint{{point[0], point[1]}, bearing, velocity};
  }

  FieldPoint mirrorOnlyY(bool shouldMirror) const {
    if (shouldMirror) {
      return FieldPoint{{point[0], field_size_y - point[1]},
          pdcsu::units::degree_t{180} - bearing, velocity};
    }
    return FieldPoint{{point[0], point[1]}, bearing, velocity};
  }

  FieldPoint mirrorOnlyX(bool shouldMirror) const {
    if (shouldMirror) {
      return FieldPoint{
          {field_size_x - point[0], point[1]}, -bearing, velocity};
    }
    return FieldPoint{{point[0], point[1]}, bearing, velocity};
  }

  FieldPoint flipDirection() const {
    return FieldPoint{
        {point[0], point[1]}, bearing + pdcsu::units::degree_t{180}, velocity};
  }

  static constexpr pdcsu::units::inch_t field_size_y =
      pdcsu::units::inch_t{690.875};
  static constexpr pdcsu::units::inch_t field_size_x =
      pdcsu::units::inch_t{317};
};

}  // namespace funkit::math