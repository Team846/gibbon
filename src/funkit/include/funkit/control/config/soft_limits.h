#pragma once

#include <cassert>

#include "pdcsu_units.h"

namespace funkit::control::config {

/*
SoftLimits

A non-templated class that implements soft limits. Meant for use by
HigherMotorController.
*/
class SoftLimits {
public:
  SoftLimits(bool using_limits, pdcsu::units::radian_t forward_limit,
      pdcsu::units::radian_t reverse_limit,
      pdcsu::units::radian_t forward_reduce,
      pdcsu::units::radian_t reverse_reduce, double reduce_max_dc);

  pdcsu::units::radian_t LimitPosition(pdcsu::units::radian_t position);

  pdcsu::units::radps_t LimitVelocity(
      pdcsu::units::radps_t velocity, pdcsu::units::radian_t position);

  double LimitDC(double dc, pdcsu::units::radian_t position);

  bool using_limits_;

private:
  pdcsu::units::radian_t forward_limit_;
  pdcsu::units::radian_t reverse_limit_;
  pdcsu::units::radian_t forward_reduce_;
  pdcsu::units::radian_t reverse_reduce_;
  double reduce_max_dc_;
};

/*
SoftLimitsHelper

A templated class to help create SoftLimits objects from system units.
*/
template <typename T> class SoftLimitsHelper {
public:
  using pos_unit = T;
  using conv_unit = pdcsu::units::UnitDivision<T, pdcsu::units::rotation_t>;

  static SoftLimits CreateSoftLimits(conv_unit conversion, bool using_limits,
      pos_unit forward_limit, pos_unit reverse_limit, pos_unit forward_reduce,
      pos_unit reverse_reduce, double reduce_max_dc) {
    return SoftLimits{using_limits, forward_limit / conversion,
        reverse_limit / conversion, forward_reduce / conversion,
        reverse_reduce / conversion, reduce_max_dc};
  }
};

}  // namespace funkit::control::config