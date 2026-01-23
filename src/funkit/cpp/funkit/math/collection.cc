#include "funkit/math/collection.h"

#include <cmath>

#include "pdcsu_units.h"

namespace funkit::math {

bool DEquals(double x, double y, double epsilon) {
  return std::abs(x - y) < epsilon;
}

double HorizontalDeadband(double input, double x_intercept, double max,
    double exponent, double sensitivity) {
  double y = 0;

  auto slope = max / (max - x_intercept);
  if (input > x_intercept) {
    y = (input - x_intercept) * slope;
  } else if (input < -x_intercept) {
    y = (input + x_intercept) * slope;
  }

  return copysign(max * pow(y / max, exponent) * sensitivity, input);
}

[[maybe_unused]] double VerticalDeadband(double input, double y_intercept,
    double max, double exponent, double sensitivity) {
  double y = 0;

  auto slope = (max - y_intercept) / max;
  if (input > 0) {
    y = input * slope + y_intercept;
  } else if (input < 0) {
    y = input * slope - y_intercept;
  }

  return copysign(max * pow(y / max, exponent) * sensitivity, input);
}

[[maybe_unused]] pdcsu::units::degree_t CoterminalDifference(
    pdcsu::units::degree_t angle, pdcsu::units::degree_t other_angle) {
  double angle_deg = angle.value();
  double other_deg = other_angle.value();
  double diff = std::fmod(angle_deg, 360.0) - std::fmod(other_deg, 360.0);
  if (diff > 180.0) {
    return pdcsu::units::degree_t{diff - 360.0};
  } else if (diff < -180.0) {
    return pdcsu::units::degree_t{diff + 360.0};
  } else {
    return pdcsu::units::degree_t{diff};
  }
}

[[maybe_unused]] pdcsu::units::degree_t CoterminalSum(
    pdcsu::units::degree_t angle, pdcsu::units::degree_t other_angle) {
  double angle_deg = angle.value();
  double other_deg = other_angle.value();
  double sum = std::fmod(angle_deg, 360.0) + std::fmod(other_deg, 360.0);
  if (sum > 180.0) {
    return pdcsu::units::degree_t{sum - 360.0};
  } else if (sum < -180.0) {
    return pdcsu::units::degree_t{sum + 360.0};
  } else {
    return pdcsu::units::degree_t{sum};
  }
}

[[maybe_unused]] pdcsu::units::degree_t modulo(
    pdcsu::units::degree_t a, pdcsu::units::degree_t b) {
  double result = std::fmod(a.value(), b.value());
  return result < 0.0 ? pdcsu::units::degree_t{result + b.value()}
                      : pdcsu::units::degree_t{result};
}

int gcd(int a, int b) {
  while (b != 0) {
    int temp = b;
    b = a % b;
    a = temp;
  }
  return std::abs(a);
}

int lcm(int a, int b) { return std::abs(a * b) / gcd(a, b); }

}  // namespace funkit::math