#pragma once
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <utility>
#include <vector>

#include "funkit/math/RampRateLimiter.h"
#include "funkit/robot/GenericCommand.h"
#include "pdcsu_control.h"
#include "subsystems/robot_constants.h"
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
  funkit::math::RampRateLimiter rampRateLimiter_x_;
  funkit::math::RampRateLimiter rampRateLimiter_y_;
  fps_t ema_comp_gpd_;
};
