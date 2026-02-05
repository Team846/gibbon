#include "funkit/control/SupremeLimiter.h"

#include "funkit/control/calculators/CurrentTorqueCalculator.h"
#include "pdcsu_control.h"

namespace funkit::control {

static constexpr double ema_growth_factor = 0.98;
static constexpr double current_limit_growth_factor = 0.95;

volt_t SupremeLimiter::ema_v_batt_ = 0_V_;
amp_t SupremeLimiter::ema_total_current_ = 0_A_;
size_t SupremeLimiter::current_samples_ = 0;

amp_t SupremeLimiter::current_limit_ = 250_A_;

static amp_t ema_current_threshold = 50_A_;
static volt_t tolerable_drop = 4.0_V_;

std::map<size_t, double> SupremeLimiter::Limit(
    std::vector<PerDeviceInformation> inputs, volt_t v_batt) {
  amp_t total_current = 0_A_;
  std::vector<std::pair<size_t, amp_t>> draws_by_index;
  std::map<size_t, double> dcs_by_index;
  std::map<size_t, PerDeviceInformation> info_by_index;

  amp_t total_limitable_current = 0_A_;
  for (PerDeviceInformation input : inputs) {
    pdcsu::util::BasePlant plant = input.plant;
    const pdcsu::units::amp_t draw =
        u_abs(calculators::CurrentTorqueCalculator::predict_current_supply(
            input.DC, input.speed, pdcsu::units::volt_t{12.0}, 0_ohm_,
            plant.def_bldc.free_speed, plant.def_bldc.stall_current));
    total_current += draw;
    if (input.is_limitable) { total_limitable_current += draw; }
    draws_by_index.push_back({input.device_id, draw});
    dcs_by_index[input.device_id] = input.DC;
    info_by_index.emplace(input.device_id, input);
  }

  ema_v_batt_ =
      ema_growth_factor * ema_v_batt_ + (1 - ema_growth_factor) * v_batt;
  ema_total_current_ = ema_growth_factor * ema_total_current_ +
                       (1 - ema_growth_factor) * total_current;

  if (current_samples_ < 100U) {
    current_samples_++;
  } else if (ema_total_current_ > ema_current_threshold) {
    amp_t new_limit =
        ema_total_current_ / (13.0_V_ - ema_v_batt_) * tolerable_drop;

    current_limit_ = current_limit_growth_factor * current_limit_ +
                     (1 - current_limit_growth_factor) * new_limit;
  }

  const double scale_adjustment =
      total_limitable_current.value() / total_current.value();
  const double original_scale_factor =
      current_limit_.value() / total_current.value();

  if (original_scale_factor >= 1.0) return dcs_by_index;
  const double scale_factor =
      original_scale_factor * std::max(0.5, scale_adjustment);

  for (const auto& draw : draws_by_index) {
    const PerDeviceInformation info = info_by_index.at(draw.first);
    const double dc =
        calculators::CurrentTorqueCalculator::scale_current_supply(scale_factor,
            dcs_by_index[draw.first], info.speed, 12.0_V_,
            info.plant.def_bldc.free_speed);
    dcs_by_index[draw.first] = dc;
  }

  return dcs_by_index;
}

}  // namespace funkit::control