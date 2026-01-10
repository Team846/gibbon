#pragma once

#include <hal/HALBase.h>

#include "pdcsu_units.h"

namespace funkit {
namespace wpilib {

// Get the current time.
pdcsu::units::second_t CurrentFPGATime();

}  // namespace wpilib
}  // namespace funkit
