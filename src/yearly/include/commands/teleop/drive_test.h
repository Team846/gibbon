#pragma once

#include <frc/Timer.h>

#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class DriveTest
    : public funkit::robot::GenericCommand<RobotContainer, DriveTest> {
public:
  DriveTest(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
  // timer
  frc::Timer timer;
  double timer_count;
  int cntr_{0};
  pdcsu::util::math::uVec<pdcsu::units::fps_t, 2u> highestVel;
  pdcsu::units::fps2_t highestAccel;
  funkit::robot::calculators::Vector2D start_pos_;
  funkit::robot::calculators::Vector2D current_pos_;
};
