#pragma once

#include "pdcsu_units.h"

namespace funkit::math {

// Double comparison using an epsilon value. Default epsilon value is 1e-9.
bool DEquals(double x, double y, double epsilon = 1e-9);

// Find the circumference of a circle given radius.
constexpr pdcsu::units::inch_t Circumference(pdcsu::units::meter_t radius) {
  return pdcsu::units::inch_t{
      2.0 * 3.14159265358979323846 * radius.value() * 39.3701};
}

double HorizontalDeadband(double input, double x_intercept, double max,
    double exponent = 1, double sensitivity = 1);

double VerticalDeadband(double input, double y_intercept, double max,
    double exponent = 1, double sensitivity = 1);

// Returns the smallest difference between angle to other_angle.
[[maybe_unused]] pdcsu::units::degree_t CoterminalDifference(
    pdcsu::units::degree_t angle, pdcsu::units::degree_t other_angle);

// Returns the smallest sum of two angles.
[[maybe_unused]] pdcsu::units::degree_t CoterminalSum(
    pdcsu::units::degree_t angle, pdcsu::units::degree_t other_angle);

// Returns a modulo b, always positive.
[[maybe_unused]] pdcsu::units::degree_t modulo(
    pdcsu::units::degree_t a, pdcsu::units::degree_t b);

// Returns the greatest common divisor of two numbers.
int gcd(int a, int b);

// Returns the least common multiple of two numbers.
int lcm(int a, int b);

}  // namespace funkit::math