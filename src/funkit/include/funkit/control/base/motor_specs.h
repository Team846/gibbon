#pragma once

#include "funkit/control/base/motor_control_base.h"
#include "pdcsu_units.h"

namespace funkit::control::base {

/*
MotorSpecs
  - free_speed: rpm
  - stall_current: amps
  - free_current: amps
  - stall_torque: Nm
  - winding_resistance: ohms

Values can be found on specification sheets for the respective motors.
*/
struct MotorSpecs {
  pdcsu::units::rpm_t free_speed;
  pdcsu::units::amp_t stall_current;
  pdcsu::units::amp_t free_current;
  pdcsu::units::nm_t stall_torque;
};

/*
MotorSpecificationPresets

Includes MotorSpecs for:
  - NEO 550 (https://www.revrobotics.com/rev-21-1651/)
  - NEO (https://www.revrobotics.com/rev-21-1650/)
  - NEO Vortex (https://www.revrobotics.com/rev-21-1652/)
  - Kraken X60
(https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance)
  - Kraken X44
(https://docs.wcproducts.com/kraken-x44/kraken-x44-motor/overview-and-features/motor-performance)

*/
struct MotorSpecificationPresets {
  static constexpr MotorSpecs kNeo550 = {
      .free_speed = pdcsu::units::rpm_t{11000},
      .stall_current = pdcsu::units::amp_t{100},
      .free_current = pdcsu::units::amp_t{1.4},
      .stall_torque = pdcsu::units::nm_t{0.97},
  };
  static constexpr MotorSpecs kNeo = {
      .free_speed = pdcsu::units::rpm_t{5676},
      .stall_current = pdcsu::units::amp_t{105},
      .free_current = pdcsu::units::amp_t{1.8},
      .stall_torque = pdcsu::units::nm_t{2.6},
  };
  static constexpr MotorSpecs kNeoVortex = {
      .free_speed = pdcsu::units::rpm_t{6784},
      .stall_current = pdcsu::units::amp_t{211},
      .free_current = pdcsu::units::amp_t{3.6},
      .stall_torque = pdcsu::units::nm_t{3.6},
  };
  static constexpr MotorSpecs kKrakenX60 = {
      .free_speed = pdcsu::units::rpm_t{6000},
      .stall_current = pdcsu::units::amp_t{366},
      .free_current = pdcsu::units::amp_t{2},
      .stall_torque = pdcsu::units::nm_t{7.09},
  };
  static constexpr MotorSpecs kKrakenX44 = {
      .free_speed = pdcsu::units::rpm_t{7530},
      .stall_current = pdcsu::units::amp_t{275},
      .free_current = pdcsu::units::amp_t{1.4},
      .stall_torque = pdcsu::units::nm_t{4.05},
  };

  static MotorSpecs get(funkit::control::base::MotorMonkeyType type) {
    switch (type) {
    case funkit::control::base::MotorMonkeyType::SPARK_MAX_NEO550:
      return kNeo550;
    case funkit::control::base::MotorMonkeyType::SPARK_MAX_NEO: return kNeo;
    case funkit::control::base::MotorMonkeyType::SPARK_FLEX_VORTEX:
    case funkit::control::base::MotorMonkeyType::SPARK_MAX_VORTEX:
      return kNeoVortex;
    case funkit::control::base::MotorMonkeyType::TALON_FX_KRAKENX60:
      return kKrakenX60;
    case funkit::control::base::MotorMonkeyType::TALON_FX_KRAKENX44:
      return kKrakenX44;
    default: return kNeo;
    }
  }
};

}  // namespace funkit::control::base
