#pragma once

#include <array>

#include "funkit/math/calculator.h"
#include "pdcsu_units.h"
#include "util/math/uvec.h"

namespace funkit::robot::swerve::control {

struct SwerveOpenLoopCalculatorConstants {
  pdcsu::units::inch_t wheelbase_horizontal_dim;
  pdcsu::units::inch_t wheelbase_forward_dim;

  pdcsu::units::radps_t rotation_iter_dec = pdcsu::units::radps_t{6};
};

struct SwerveOpenLoopCalculatorInputs {
  pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> translation_target;
  pdcsu::units::degps_t rotation_target;
  pdcsu::units::degree_t bearing;
  pdcsu::units::fps_t max_speed;
  bool cut_excess_steering;
};

struct SwerveOpenLoopCalculatorOutput {
  std::array<pdcsu::units::fps_t, 4> drive_outputs;
  std::array<pdcsu::units::degree_t, 4> steer_outputs;
};

/*
SwerveOpenLoopCalculator

Calculates the open-loop control targets for each swerve module, given target
translational and rotational velocities.
*/
class SwerveOpenLoopCalculator
    : public funkit::math::Calculator<SwerveOpenLoopCalculatorInputs,
          SwerveOpenLoopCalculatorOutput, SwerveOpenLoopCalculatorConstants> {
public:
  SwerveOpenLoopCalculatorOutput calculate(
      SwerveOpenLoopCalculatorInputs inputs) override;
};

}  // namespace funkit::robot::swerve::control
