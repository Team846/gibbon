#pragma once

#include <units/length.h>

#include "pdcsu_units.h"

namespace funkit::control::calculators {

using ohms_per_meter_t =
    pdcsu::units::UnitDivision<pdcsu::units::ohm_t, pdcsu::units::meter_t>;

struct KnownResistances {
  static constexpr pdcsu::units::ohm_t kBatteryResistance{0.02};
  static constexpr pdcsu::units::ohm_t kPDPResistance{0.001};

  static constexpr pdcsu::units::ohm_t kConnectorResistance{0.0006};

  static constexpr ohms_per_meter_t kTwelveGaugeResistance{0.00521};
  static constexpr ohms_per_meter_t kFourteenGaugeResistance{0.00829};
  static constexpr ohms_per_meter_t kSixteenGaugeResistance{0.0132};
  static constexpr ohms_per_meter_t kEighteenGaugeResistance{0.0209};
};

enum WireGauge { twelve_gauge, fourteen_gauge, sixteen_gauge, eighteen_gauge };

class CircuitResistanceCalculator {
public:
  static constexpr pdcsu::units::ohm_t calculate(
      pdcsu::units::foot_t wire_length, WireGauge gauge,
      unsigned int num_connectors) {
    ohms_per_meter_t resistance_per_meter;
    switch (gauge) {
    case twelve_gauge:
      resistance_per_meter = KnownResistances::kTwelveGaugeResistance;
      break;
    case fourteen_gauge:
      resistance_per_meter = KnownResistances::kFourteenGaugeResistance;
      break;
    case sixteen_gauge:
      resistance_per_meter = KnownResistances::kSixteenGaugeResistance;
      break;
    case eighteen_gauge:
      resistance_per_meter = KnownResistances::kEighteenGaugeResistance;
      break;
    default:
      resistance_per_meter = KnownResistances::kEighteenGaugeResistance;
      break;
    }

    pdcsu::units::ohm_t total_resistance =
        resistance_per_meter * wire_length + KnownResistances::kPDPResistance +
        KnownResistances::kBatteryResistance +
        KnownResistances::kConnectorResistance * num_connectors;

    return total_resistance;
  }
};

}  // namespace funkit::control::calculators