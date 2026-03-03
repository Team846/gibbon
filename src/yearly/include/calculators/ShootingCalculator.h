#pragma once

#include "funkit/base/Loggable.h"
#include "funkit/math/fieldpoints.h"
#include "pdcsu_units.h"
#include "subsystems/robot_container.h"

struct ShootingCalculatorOutputs {
  /* For drivetrain */
  pdcsu::units::degree_t aim_angle;
  pdcsu::units::radps_t vel_aim_compensation;

  /* For shooter */
  pdcsu::units::fps_t shooter_vel;

  /* General */
  bool is_valid;
};

struct ShootingCalculatorInput {
  funkit::math::Vector2D position;
  pdcsu::util::math::uVec<pdcsu::units::fps_t, 2> velocity;
  pdcsu::units::degree_t bearing;
  pdcsu::units::degps_t yaw_rate;
  bool is_blue_alliance;
};

class ShootingCalculator {
public:
  static void Setup();

  static void Calculate(const RobotContainer* container_);

  static ShootingCalculatorOutputs GetOutputs() { return outputs_; };

  static ShootingCalculatorOutputs CalculateFromInput(
      const ShootingCalculatorInput& in);

private:
  static ShootingCalculatorOutputs outputs_;
  static std::optional<funkit::base::Loggable> loggable_opt;
};
