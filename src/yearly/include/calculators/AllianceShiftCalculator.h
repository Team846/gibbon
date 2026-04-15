#pragma once

#include <string>

struct AllianceShiftOutputs {
  bool our_hub_active;
  std::string auto_winner;
  int phase_idx;
  double phase_countdown;
  double total_countdown;
};

class AllianceShiftCalculator {
public:
  static AllianceShiftOutputs Calculate();
  static bool shot_valid;
};