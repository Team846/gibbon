#include "calculators/TurretPositionCalculator.h"

#include <cmath>
#include <limits>
#include <vector>

#include "funkit/math/collection.h"

using namespace pdcsu::units;

TurretPositionCalculator::CrtSolution TurretPositionCalculator::GetPosition(
    CrtInputs inputs) {
  std::vector<rotation_t> candidatesA;
  std::vector<rotation_t> candidatesB;
  double ratioA = inputs.mainTeeth / inputs.teethA;
  double ratioB = inputs.mainTeeth / inputs.teethB;
  rotation_t period{
      funkit::math::lcm(inputs.teethA, inputs.teethB) / inputs.mainTeeth};
  rotation_t searchMax = inputs.maxRots;
  if (period < searchMax) { searchMax = period; }
  int minN_A = u_floor((inputs.minRots * ratioA) - inputs.absA).value();
  int maxN_A = u_ceil((searchMax * ratioA) - inputs.absA).value();
  minN_A -= 1;
  maxN_A += 1;
  for (int n = minN_A; n <= maxN_A; ++n) {
    double t_val = (static_cast<double>(n) + inputs.absA.value()) / ratioA;
    rotation_t t_rot{t_val};
    if (t_rot >= (inputs.minRots - inputs.maxTol) &&
        t_rot <= (inputs.maxRots + inputs.maxTol)) {
      candidatesA.push_back(t_rot);
    }
  }
  int minN_B = u_floor((inputs.minRots * ratioB) - inputs.absB).value();
  int maxN_B = u_ceil((searchMax * ratioB) - inputs.absB).value();
  minN_B -= 1;
  maxN_B += 1;
  for (int n = minN_B; n <= maxN_B; ++n) {
    double t_val = (static_cast<double>(n) + inputs.absB.value()) / ratioB;
    rotation_t t_rot{t_val};
    if (t_rot >= (inputs.minRots - inputs.maxTol) &&
        t_rot <= (inputs.maxRots + inputs.maxTol)) {
      candidatesB.push_back(t_rot);
    }
  }
  CrtSolution bestSol{
      rotation_t{0.0}, rotation_t{std::numeric_limits<double>::max()}, false};

  for (const auto& posA : candidatesA) {
    for (const auto& posB : candidatesB) {
      rotation_t diff = u_abs(posA - posB);
      if (diff < inputs.maxTol) {
        if (diff < bestSol.error) {
          bestSol.error = diff;
          bestSol.turretRotations = (posA + posB) / 2.0;
          bestSol.isValid = true;
        }
      }
    }
  }

  if (bestSol.isValid) {
    if (bestSol.turretRotations < inputs.minRots ||
        bestSol.turretRotations > inputs.maxRots) {
      bestSol.isValid = false;
    }
  }

  return bestSol;
}