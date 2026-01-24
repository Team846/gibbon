#include "funkit/control/hardware/Cooked.h"

#include "funkit/wpilib/time.h"

namespace funkit::control::hardware {
Cooked::Cooked(CookedConfig config) : config_(config) {}

amp_t Cooked::Record(amp_t current, radps_t speed, double measured_temp) {
  current = u_abs(current);

  if (measured_temp > thresh_meas_temp) current *= meas_temp_cut_scale;

  scalar_t delta_temp{0.0};
  second_t new_time = funkit::wpilib::CurrentFPGATime();
  second_t delta_time = new_time - time_;
  time_ = new_time;
  for (int i = 0; i < 3; i++) {  // Iteratively solve for delta_temp
    watt_t power = current * (speed / config_.free_speed_) * 12_V_;
    delta_temp = power /
                 UnitDivision<joule_t, scalar_t>(config_.thermal_mass_) *
                 delta_time;

    second_t time_to_cook =
        (cook_temp - temp_) / power *
        UnitDivision<joule_t, scalar_t>(config_.thermal_mass_);
    if (time_to_cook > min_cook_time) current *= min_cook_time / time_to_cook;
  }

  temp_ += delta_temp.value();
  double time_exp = std::pow(config_.growth_rate_, delta_time.value());
  temp_ = temp_ * time_exp + atm_temp * (1 - time_exp);

  double tre_exp = std::pow(temp_relocalize_ema_growth, delta_time.value());
  temp_ = temp_ * tre_exp + measured_temp * (1 - tre_exp);

  return current;
}

double Cooked::Record(double dc, radps_t speed, double measured_temp) {
  double speed_pct = (speed / config_.free_speed_).value();
  amp_t draw = config_.stall_current_ * (dc - speed_pct);
  amp_t newdraw = Record(draw, speed, measured_temp);
  return (u_copysign(newdraw, draw) / config_.stall_current_).value() +
         speed_pct;
}
}