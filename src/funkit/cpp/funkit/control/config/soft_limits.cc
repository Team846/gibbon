#include "funkit/control/config/soft_limits.h"

#include "pdcsu_units.h"

namespace funkit::control::config {

SoftLimits::SoftLimits(bool using_limits, pdcsu::units::radian_t forward_limit,
    pdcsu::units::radian_t reverse_limit, pdcsu::units::radian_t forward_reduce,
    pdcsu::units::radian_t reverse_reduce, double reduce_max_dc)
    : using_limits_(using_limits),
      forward_limit_(forward_limit),
      reverse_limit_(reverse_limit),
      forward_reduce_(forward_reduce),
      reverse_reduce_(reverse_reduce),
      reduce_max_dc_(reduce_max_dc) {
  // assert(forward_limit >= reverse_limit &&
  //        "Forward limit must be greater than reverse limit");
  // assert(forward_reduce_ <= forward_limit_ &&
  //        "Forward reduce must be less than forward limit");
  // assert(reverse_reduce_ >= reverse_limit_ &&
  //        "Reverse reduce must be greater than reverse limit");
  // assert(forward_reduce_ >= reverse_reduce_ &&
  //        "Forward reduce must be greater than reverse reduce");
  // assert(reduce_max_dc_ > 0 && "Reduce max DC must be greater than 0");
}

pdcsu::units::radian_t SoftLimits::LimitPosition(
    pdcsu::units::radian_t position) {
  if (using_limits_ && position.value() < reverse_limit_.value()) {
    return reverse_limit_;
  } else if (using_limits_ && position.value() > forward_limit_.value()) {
    return forward_limit_;
  }
  return position;
}

pdcsu::units::radps_t SoftLimits::LimitVelocity(
    pdcsu::units::radps_t velocity, pdcsu::units::radian_t position) {
  if (!using_limits_) return velocity;

  if (velocity.value() < 0.0 && position.value() <= reverse_limit_.value()) {
    return pdcsu::units::radps_t{0.0};
  } else if (velocity.value() > 0.0 &&
             position.value() >= forward_limit_.value()) {
    return pdcsu::units::radps_t{0.0};
  }

  return velocity;
}

double SoftLimits::LimitDC(double dc, pdcsu::units::radian_t position) {
  if (!using_limits_) return dc;

  if (dc < 0 && position.value() <= reverse_limit_.value()) { return 0.0; }
  if (dc > 0 && position.value() >= forward_limit_.value()) { return 0.0; }

  if (position.value() > forward_reduce_.value() && dc > reduce_max_dc_) {
    return reduce_max_dc_;
  }
  if (position.value() < reverse_reduce_.value() && dc < -reduce_max_dc_) {
    return -reduce_max_dc_;
  }

  return dc;
}

}  // namespace funkit::control::config