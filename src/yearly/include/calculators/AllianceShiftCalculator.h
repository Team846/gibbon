#pragma once

struct AllianceShiftOutputs {
  bool our_hub_active;
  bool won_auto;
  int shift;
  double until_flip;
  double our_time_left;
};

class AllianceShiftCalculator {
public:
  static AllianceShiftOutputs Calculate();
};