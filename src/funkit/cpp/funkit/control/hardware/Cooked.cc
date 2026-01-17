#include "funkit/control/hardware/Cooked.h"

#include "funkit/wpilib/time.h"

namespace funkit::control::hardware {
Cooked::Cooked(CookedConfig config) : config_(config) {}

amp_t Cooked::RecordDraw(amp_t current, radps_t speed) {
  current = u_abs(current);

  scalar_t delta_temp{0.0};
  second_t new_time = funkit::wpilib::CurrentFPGATime();
  second_t delta_time = new_time - time_;
  time_ = new_time;
  for (int i = 0; i < 3; i++) {  // Iteratively solve for delta_temp
    watt_t power = current * (speed / config_.free_speed_) * 12_u_V;
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

  return current;
}

double Cooked::RecordDraw(double dc, radps_t speed) {
  double speed_pct = (speed / config_.free_speed_).value();
  amp_t draw = config_.stall_current_ * (dc - speed_pct);
  amp_t newdraw = RecordDraw(draw, speed);
  return (u_copysign(newdraw, draw) / config_.stall_current_).value() +
         speed_pct;
}
}