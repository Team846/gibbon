#pragma once

#include <array>

#include "pdcsu_units.h"

namespace funkit::math {

struct Quaternion {
  double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
  static Quaternion FromPitchRoll(
      pdcsu::units::degree_t pitch, pdcsu::units::degree_t roll) {
    Quaternion q_pitch{u_cos(pitch / 2.0), u_sin(pitch / 2.0), 0.0, 0.0};
    Quaternion q_roll{u_cos(roll / 2.0), 0.0, u_sin(roll / 2.0), 0.0};
    return q_roll * q_pitch;
  }

  Quaternion operator*(const Quaternion& o) const {
    return {w * o.w - x * o.x - y * o.y - z * o.z,
        w * o.x + x * o.w + y * o.z - z * o.y,
        w * o.y - x * o.z + y * o.w + z * o.x,
        w * o.z + x * o.y - y * o.x + z * o.w};
  }

  Quaternion Conjugate() const { return {w, -x, -y, -z}; }

  // Rotates a vector by this quaternion.
  std::array<double, 3> Rotate(std::array<double, 3> v) const {
    Quaternion p{0.0, v[0], v[1], v[2]};
    Quaternion r = (*this) * p * Conjugate();
    return {r.x, r.y, r.z};
  }
};

}  // namespace funkit::math
