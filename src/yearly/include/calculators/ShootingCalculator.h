#pragma once

#include "funkit/base/Loggable.h"
#include "pdcsu_units.h"
#include "subsystems/robot_container.h"

struct ShootingCalculatorOutputs {
  /* For drivetrain */
  pdcsu::units::degree_t aim_angle;
  pdcsu::units::radps_t vel_aim_compensation;

  /* For hood */
  pdcsu::units::degree_t shot_angle;
  pdcsu::units::radps_t shot_angle_vel;

  /* For shooter */
  pdcsu::units::fps_t shooter_vel;

  /* General */
  bool is_valid;

  /* For trajectory simulation */
  pdcsu::util::math::Vector2D start_traj{-1000_in_, -1000_in_};
  pdcsu::util::math::Vector2D term_traj{-1100_in_, -1100_in_};
};

class ShootingCalculator {
public:
  static void Setup();

  static void Calculate(const RobotContainer* container_);

  static degree_t GetShotAngle(foot_t shot_distance);
  static radps_t GetShotAngleVel(foot_t shot_distance, fps_t vel_in_dir);
  static fps_t GetBaseVelocity(degree_t shot_angle, foot_t shot_distance);

  static ShootingCalculatorOutputs GetOutputs() { return outputs_; };
  static pdcsu::util::math::Vector2D SimulateTrajectory(
      const RobotContainer* container_);

  static pdcsu::util::math::Vector2D target;

private:
  static ShootingCalculatorOutputs outputs_;
  static std::optional<funkit::base::Loggable> loggable_opt;
};
