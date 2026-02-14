#pragma once

#include "funkit/base/Loggable.h"
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

class ShootingCalculator {
public:
  static void Setup();

  static void Calculate(const RobotContainer* container_);

  static ShootingCalculatorOutputs GetOutputs() { return outputs_; };

  static pdcsu::util::math::Vector2D target;

private:
  static ShootingCalculatorOutputs outputs_;
  static std::optional<funkit::base::Loggable> loggable_opt;
};
