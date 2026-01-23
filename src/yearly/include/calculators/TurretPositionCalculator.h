#pragma once

#include "pdcsu_units.h"

class TurretPositionCalculator {
public:
  struct CrtSolution {
    pdcsu::units::rotation_t turretRotations;
    pdcsu::units::rotation_t error;
    bool isValid;
  };

  struct CrtInputs {
    pdcsu::units::rotation_t absA;
    pdcsu::units::rotation_t absB;
    double teethA;
    double teethB;
    double mainTeeth;
    pdcsu::units::rotation_t minRots;
    pdcsu::units::rotation_t maxRots;
    pdcsu::units::rotation_t maxTol;
  };

  static CrtSolution GetPosition(CrtInputs inputs);
};
