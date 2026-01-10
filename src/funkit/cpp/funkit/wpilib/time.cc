#include "funkit/wpilib/time.h"

#include "pdcsu_units.h"

namespace funkit::wpilib {

pdcsu::units::second_t CurrentFPGATime() {
  int err;

  return pdcsu::units::ms_t(HAL_GetFPGATime(&err) / 1000.0);
}

}  // namespace funkit::wpilib