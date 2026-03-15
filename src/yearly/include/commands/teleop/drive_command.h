#pragma once

#include "funkit/math/RampRateLimiter.h"
#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class DriveCommand
    : public funkit::robot::GenericCommand<RobotContainer, DriveCommand> {
public:
  DriveCommand(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
  fps_t ema_comp_gpd_;

  funkit::math::RampRateLimiter ramp_rate_limiter_x_{};
  funkit::math::RampRateLimiter ramp_rate_limiter_y_{};
};