#pragma once

#include <cassert>

#include "pdcsu_units.h"

namespace funkit::control::config {

/**
 * SoftLimits
 * 
 * A non-templated class that implements soft limits. 
 * Soft limits are limits to a subsystems movement to prevent any potential mechanical damage. Meant for use by HigherMotorController.
 */
class SoftLimits {
public:
  /**
   * SoftLimits()
   * 
   * Implemented in pure rotational units to match plant's native coordinates. 
   * @param using_limits - if limits are being used
   * @param forward_limit - positive hard boundary on position
   * @param reverse_limit - negative hard boundary on position
   * @param forward_reduce - positive soft boundary on position
   * @param reverse_reduce - negative soft boundary on position
   * @param reduce_max_dc - maximum duty cycle allowed
   */
  SoftLimits(bool using_limits, pdcsu::units::radian_t forward_limit,
      pdcsu::units::radian_t reverse_limit,
      pdcsu::units::radian_t forward_reduce,
      pdcsu::units::radian_t reverse_reduce, double reduce_max_dc);

  // Limits position if value goes beyond the forward/reverse limits. Otherwise returns original position.
  pdcsu::units::radian_t LimitPosition(pdcsu::units::radian_t position);

  // Limits velocity to zero if position goes beyond the forward/reverse limits. Otherwise returns original velocity.
  pdcsu::units::radps_t LimitVelocity(
      pdcsu::units::radps_t velocity, pdcsu::units::radian_t position);

  // Limits the duty cycle to zero if duty cycle would continue to push beyond hard limits. 
  // Otherwise, limits the duty cycle to maximum given value if beyond soft limits. 
  double LimitDC(double dc, pdcsu::units::radian_t position);

  bool using_limits_;

private:
  pdcsu::units::radian_t forward_limit_;
  pdcsu::units::radian_t reverse_limit_;
  pdcsu::units::radian_t forward_reduce_;
  pdcsu::units::radian_t reverse_reduce_;
  double reduce_max_dc_;
};

/**
 * SoftLimitsHelper
 * 
 * A templated class to help create SoftLimits objects from system units through conversions. 
 */
template <typename T> class SoftLimitsHelper {
public:
  using pos_unit = T;
  using conv_unit = pdcsu::units::UnitDivision<T, pdcsu::units::rotation_t>;

  /**
   * CreateSoftLimits()
   * 
   * @return new soft limits given a unit conversion. Keeps using_limits and reduce_max_dc the same. 
   */
  static SoftLimits CreateSoftLimits(conv_unit conversion, bool using_limits,
      pos_unit forward_limit, pos_unit reverse_limit, pos_unit forward_reduce,
      pos_unit reverse_reduce, double reduce_max_dc) {
    return SoftLimits{using_limits, forward_limit / conversion,
        reverse_limit / conversion, forward_reduce / conversion,
        reverse_reduce / conversion, reduce_max_dc};
  }
};

}  // namespace funkit::control::config