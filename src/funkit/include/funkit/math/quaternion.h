#pragma once

#include <array>
#include <cmath>

#include "pdcsu_units.h"

namespace funkit::math {

struct Quaternion {
  double w;
  double x;
  double y;
  double z;

  static Quaternion Conjugate(const Quaternion& q) {
    return Quaternion{q.w, -q.x, -q.y, -q.z};
  }

  static Quaternion Multiply(const Quaternion& a, const Quaternion& b) {
    return Quaternion{
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
  }
};

inline Quaternion RotationAboutX(pdcsu::units::degree_t angle) {
  const auto half = angle / 2.0;
  const double s = pdcsu::units::u_sin(half);
  const double c = pdcsu::units::u_cos(half);
  return Quaternion{c, s, 0.0, 0.0};
}

inline Quaternion RotationAboutY(pdcsu::units::degree_t angle) {
  const auto half = angle / 2.0;
  const double s = pdcsu::units::u_sin(half);
  const double c = pdcsu::units::u_cos(half);
  return Quaternion{c, 0.0, s, 0.0};
}

inline Quaternion RotationAboutZ(pdcsu::units::degree_t angle) {
  const auto half = angle / 2.0;
  const double s = pdcsu::units::u_sin(half);
  const double c = pdcsu::units::u_cos(half);
  return Quaternion{c, 0.0, 0.0, s};
}

inline std::array<double, 3> RotateVector(
    const Quaternion& q, const std::array<double, 3>& v) {
  const Quaternion p{0.0, v[0], v[1], v[2]};
  const Quaternion qp = Quaternion::Multiply(
      Quaternion::Multiply(q, p), Quaternion::Conjugate(q));
  return {qp.x, qp.y, qp.z};
}

inline pdcsu::units::degree_t ElevationAngle(const std::array<double, 3>& v) {
  const double horiz = std::hypot(v[0], v[1]);
  return pdcsu::units::degree_t{
      pdcsu::units::radian_t{std::atan2(v[2], horiz)}};
}

}  // namespace funkit::math
