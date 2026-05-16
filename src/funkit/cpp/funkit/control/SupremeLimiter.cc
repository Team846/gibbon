#include "funkit/control/SupremeLimiter.h"

#include <algorithm>

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
  const volt_t safe_v_batt = u_max(v_batt, 1.0_V_);
  const double command_normalization_gain =
      std::clamp((12.0_V_ / safe_v_batt).value(), 0.5, 1.5);
  const volt_t v_cmd_effective = command_normalization_gain * safe_v_batt;

  amp_t total_draw_current = 0_A_;
  amp_t total_regen_current = 0_A_;
  std::vector<std::pair<size_t, amp_t>> supply_by_index;
  std::map<size_t, double> dcs_by_index;
  std::map<size_t, PerDeviceInformation> info_by_index;

  amp_t total_limitable_draw_current = 0_A_;
  amp_t total_limitable_regen_current = 0_A_;
  for (PerDeviceInformation input : inputs) {
    pdcsu::util::BasePlant plant = input.plant;
    const pdcsu::units::amp_t supply =
        calculators::CurrentTorqueCalculator::predict_current_supply_signed(
            input.DC, input.speed, v_cmd_effective, input.circuit_resistance,
            plant.def_bldc.free_speed, plant.def_bldc.stall_current);
    if (supply > 0_A_) {
      total_draw_current += supply;
      if (input.is_limitable) total_limitable_draw_current += supply;
    } else {
      const amp_t regen = u_abs(supply);
      total_regen_current += regen;
      if (input.is_limitable) total_limitable_regen_current += regen;
    }
    supply_by_index.push_back({input.device_id, supply});
    dcs_by_index[input.device_id] = input.DC;
    info_by_index.emplace(input.device_id, input);
  }

  ema_v_batt_ =
      ema_growth_factor * ema_v_batt_ + (1 - ema_growth_factor) * v_batt;
  ema_total_current_ = ema_growth_factor * ema_total_current_ +
                       (1 - ema_growth_factor) * total_draw_current;

  if (current_samples_ < 100U) {
    current_samples_++;
  } else if (ema_total_current_ > ema_current_threshold) {
    amp_t new_draw_limit =
        ema_total_current_ / (13.0_V_ - ema_v_batt_) * tolerable_drop;

    current_limit_ = current_limit_growth_factor * current_limit_ +
                     (1 - current_limit_growth_factor) * new_draw_limit;
  }

  const double draw_scale_adjustment =
      total_draw_current > 0_A_
          ? (total_limitable_draw_current / total_draw_current).value()
          : 1.0;
  const double draw_original_scale_factor =
      total_draw_current > 0_A_ ? (current_limit_ / total_draw_current).value()
                                : 1.0;
  const double draw_scale_factor =
      draw_original_scale_factor * std::max(0.5, draw_scale_adjustment);

  const double regen_scale_adjustment =
      total_regen_current > 0_A_
          ? (total_limitable_regen_current / total_regen_current).value()
          : 1.0;
  const double regen_original_scale_factor =
      total_regen_current > 0_A_
          ? (current_limit_ / total_regen_current).value()
          : 1.0;
  const double regen_scale_factor =
      regen_original_scale_factor * std::max(0.5, regen_scale_adjustment);

  for (const auto& supply_entry : supply_by_index) {
    const size_t device_id = supply_entry.first;
    const amp_t supply_current = supply_entry.second;
    const PerDeviceInformation info = info_by_index.at(device_id);
    if (!info.is_limitable) continue;

    if (supply_current > 0_A_ && draw_scale_factor < 1.0) {
      const amp_t target_supply = draw_scale_factor * supply_current;
      dcs_by_index[device_id] =
          calculators::CurrentTorqueCalculator::supply_current_control(info.DC,
              target_supply, info.speed, v_cmd_effective,
              info.circuit_resistance, info.plant.def_bldc.free_speed,
              info.plant.def_bldc.stall_current);
    } else if (supply_current < 0_A_ && regen_scale_factor < 1.0) {
      const amp_t target_supply = regen_scale_factor * supply_current;
      dcs_by_index[device_id] =
          calculators::CurrentTorqueCalculator::supply_current_control(info.DC,
              target_supply, info.speed, v_cmd_effective,
              info.circuit_resistance, info.plant.def_bldc.free_speed,
              info.plant.def_bldc.stall_current);
    }
  }

  return dcs_by_index;
}

}  // namespace funkit::control