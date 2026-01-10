#include "funkit/wpilib/time.h"

#include <units/time.h>

#include "pdcsu_units.h"

namespace funkit::wpilib {

pdcsu::units::second_t CurrentFPGATime() {
  int err;
  units::microsecond_t ustime(HAL_GetFPGATime(&err));
  return pdcsu::units::second_t(ustime.value() / 1000000.0);
}

}  // namespace funkit::wpilib