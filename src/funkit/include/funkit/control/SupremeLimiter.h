#pragma once

#include <map>
#include <vector>

#include "funkit/control/base/motor_specs.h"
#include "pdcsu_control.h"
#include "pdcsu_units.h"

namespace funkit::control {

struct PerDeviceInformation {
  size_t device_id;
  pdcsu::util::BasePlant plant;
  radps_t speed;
  double DC;
};

/*
Supreme Limiter.

Manages power draw across all devices that run custom control loops on the Rio.
*/
class SupremeLimiter {
public:
  /* Limits duty cycle to implement current limit across all devices. No
   * prioritization is applied. */
  [[nodiscard]] static std::map<size_t, double> Limit(
      std::vector<PerDeviceInformation> inputs, volt_t v_batt,
      size_t num_limitable);

private:
  static volt_t ema_v_batt_;
  static amp_t ema_total_current_;
  static size_t current_samples_;

  static amp_t current_limit_;
};
}