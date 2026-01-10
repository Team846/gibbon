#pragma once
#include <variant>

#include "pdcsu_units.h"

namespace funkit::control::base {

/*
MotorMonkeyType (enum)

Contains all motor and speed controller combinations compatible with the
funkit architecture.
*/
enum MotorMonkeyType {
  TALON_FX_KRAKENX60,
  TALON_FX_KRAKENX44,
  SPARK_FLEX_VORTEX,
  SPARK_MAX_VORTEX,
  SPARK_MAX_NEO,
  SPARK_MAX_NEO550,
};

enum LimitSwitchDefaultState {
  kNormallyOn,
  kNormallyOff,
};

/*
MotorMonkeyTypeHelper

Provides static methods to help determine the type of motor controller.
*/
class MotorMonkeyTypeHelper {
public:
  static bool is_talon_fx(MotorMonkeyType mmtype) {
    switch (mmtype) {
    case TALON_FX_KRAKENX60:
    case TALON_FX_KRAKENX44: return true;
    default: return false;
    }
  }

  static bool is_spark_max(MotorMonkeyType mmtype) {
    switch (mmtype) {
    case SPARK_MAX_NEO:
    case SPARK_MAX_NEO550:
    case SPARK_MAX_VORTEX: return true;
    default: return false;
    }
  }

  static bool is_spark_flex(MotorMonkeyType mmtype) {
    switch (mmtype) {
    case SPARK_FLEX_VORTEX: return true;
    default: return false;
    }
  }
};

/*
ControlRequest

Variant type representing possible motor control requests.
*/
using ControlRequest = std::variant<double,  // duty cycle
    pdcsu::units::radps_t,                   // velocity
    pdcsu::units::radian_t>;                 // position

}  // namespace funkit::control::base