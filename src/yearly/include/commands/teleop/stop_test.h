#pragma once

#include "funkit/robot/GenericCommand.h"
#include "subsystems/robot_container.h"

class StopTest
    : public funkit::robot::GenericCommand<RobotContainer, StopTest> {
public:
  StopTest(RobotContainer& container);

  void OnInit() override;

  void Periodic() override;

  void OnEnd(bool interrupted) override;

  bool IsFinished() override;

private:
  double timer;
  funkit::robot::swerve::odometry::Vector2D start_pos;
  bool do_stop;
};