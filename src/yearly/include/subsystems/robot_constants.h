#pragma once

#include "funkit/control/calculators/CircuitResistanceCalculator.h"
#include "pdcsu_units.h"

struct robot_constants {
  struct base {
    static constexpr inch_t wheelbase_x = inch_t{22.5};
    static constexpr inch_t wheelbase_y = inch_t{25.5};
    static constexpr pound_t weight = pound_t{60};

    static constexpr inch_t height = inch_t{1.5};
  };

  static constexpr pound_t total_weight = base::weight;
};