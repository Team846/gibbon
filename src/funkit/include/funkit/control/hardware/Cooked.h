#pragma once

#include "funkit/control/base/motor_specs.h"
#include "pdcsu_units.h"

namespace funkit::control::hardware {

struct CookedConfig {
  double thermal_mass_ = 0.001;  // kg m^2 s^-2 K^-1 (J K^-1)
  double growth_rate_ = 0.99;    // s^-1
  radps_t free_speed_ = 628_u_radps;
  amp_t stall_current_ = 250_u_A;
  CookedConfig(funkit::control::base::MotorMonkeyType mmtype) {
    switch (mmtype) {
    case funkit::control::base::MotorMonkeyType::SPARK_MAX_NEO550:
      thermal_mass_ = 0.01428;
      break;
    case funkit::control::base::MotorMonkeyType::SPARK_MAX_NEO:
      thermal_mass_ = 0.05714;
      break;
    case funkit::control::base::MotorMonkeyType::SPARK_FLEX_VORTEX:
      thermal_mass_ = 0.08571;
      break;
    case funkit::control::base::MotorMonkeyType::SPARK_MAX_VORTEX:
      thermal_mass_ = 0.08571;
      break;
    default:
      throw std::runtime_error(
          "Cooked not implemented for this motor monkey type. One could say "
          "this is a cooked implementation...");
    }
    base::MotorSpecs specs = base::MotorSpecificationPresets::get(mmtype);
    free_speed_ = specs.free_speed;
    stall_current_ = specs.stall_current;
  }
};

/*
Cooked.

Utility class to keep the magic smoke within the motors.
Only works with Rev motors (Vortex, Neo, Neo550).
*/
class Cooked {
public:
  Cooked(CookedConfig config);

  // Returns the new safe current limit to prevent the motor from overcooking
  [[nodiscard]] amp_t RecordDraw(amp_t current, radps_t speed);
  [[nodiscard]] double RecordDraw(double dc, radps_t speed);

private:
  CookedConfig config_;

  static constexpr double atm_temp{20.0};
  double temp_ = atm_temp;  // deg C

  bool first = false;
  second_t time_;

  static constexpr double cook_temp{100.0};
  static constexpr second_t min_cook_time{15};
};
}