#include "calculators/AllianceShiftCalculator.h"

#include <frc/DriverStation.h>

AllianceShiftOutputs AllianceShiftCalculator::Calculate() {
  static int is_cached = -1;

  double t = frc::DriverStation::GetMatchTime().value();

  constexpr double kB[] = {125, 105, 80, 55, 30};
  int shift = (t > kB[0])   ? 0
              : (t > kB[1]) ? 1
              : (t > kB[2]) ? 2
              : (t > kB[3]) ? 3
              : (t > kB[4]) ? 4
                            : 0;

  if (is_cached == -1) {
    auto gd = frc::DriverStation::GetGameSpecificMessage();
    if (!gd.empty()) {
      bool is_blue = frc::DriverStation::GetAlliance() ==
                     frc::DriverStation::Alliance::kBlue;
      is_cached = (is_blue == (gd[0] == 'B')) ? 1 : 0;
    }
  }

  if (shift == 0 || is_cached == -1) {
    return {true, is_cached == 0, shift, 0, 0};
  }

  bool our_active = (shift % 2 == 1) != (is_cached == 1);
  double flip = t - kB[shift];

  return {our_active, is_cached == 0, shift, flip, our_active ? flip : 0};
}