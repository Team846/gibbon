#include "calculators/AllianceShiftCalculator.h"

#include <frc/DriverStation.h>

bool AllianceShiftCalculator::shot_valid = false;

AllianceShiftOutputs AllianceShiftCalculator::Calculate() {
  static bool we_won = false;
  static bool we_lost = false;
  static const char* winner = "unknown";

  if (frc::DriverStation::IsDisabled()) {
    shot_valid = false;
    winner = "unknown";
    we_won = false;
    we_lost = false;
    return {false, winner, -1, 0.0, 0.0};
  }

  double ds = frc::DriverStation::GetMatchTime().value();
  double t = frc::DriverStation::IsAutonomous() ? 20.0 - ds : 160.0 - ds;
  if (ds < 0 || t < 0) return {false, winner, -1, 0.0, 0.0};

  if (winner[0] == 'u') {
    std::string msg = frc::DriverStation::GetGameSpecificMessage();
    if (!msg.empty()) {
      winner = (msg[0] == 'B') ? "blue" : "red";
      char us = (frc::DriverStation::GetAlliance() ==
                    frc::DriverStation::Alliance::kBlue)
                    ? 'B'
                    : 'R';
      we_won = (msg[0] == us);
      we_lost = !we_won;
    }
  }

  constexpr double ends[] = {20, 30, 55, 80, 105, 130, 160};
  constexpr int owns[] = {0, 0, 1, 2, 1, 2, 0};

  int p = 0;
  while (p < 6 && t >= ends[p]) {
    p++;
  }

  auto active = [&](int i) {
    return owns[i] == 0 || (owns[i] == 1 && we_lost) ||
           (owns[i] == 2 && we_won);
  };

  bool act = active(p);
  shot_valid = act || (p > 0 && t - ends[p - 1] < 1.5 && active(p - 1)) ||
               (p < 6 && ends[p] - t <= 1.5 && active(p + 1));

  return {act, winner, p, std::max(0.0, ends[p] - t), std::max(0.0, 160.0 - t)};
}