#include "commands/teleop/stop_test.h"
#include <iostream>

/**
Records braking time from a set velocity 
 */
StopTest::StopTest(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, StopTest>{
          container, "stop_test"} {
  AddRequirements({&container_.drivetrain_});
}

void StopTest::OnInit() {
  //set target to a certain velocity (make this a pref)
  do_stop = false;
  timer = 0;
}

void StopTest::Periodic() {
  auto velocity = container_.drivetrain_.GetReadings().estimated_pose.velocity;
  funkit::robot::swerve::DrivetrainTarget target{};
  if (!do_stop) 
  {
    target.velocity = {0.0_fps_, container_.drivetrain_.GetPreferenceValue_double("duty_cycle") * 13.7_fps_};
    target.angular_velocity = 0.0_degps_;
    if (container_.drivetrain_.GetPreferenceValue_unit_type<pdcsu::util::math::uVec<pdcsu::units::fps_t, 2>>("target_velocity").magnitude() <= velocity.magnitude()) 
    {
        do_stop = true;
    }
    container_.drivetrain_.SetTarget(target);
  }
  else {
    timer += 0.01;
    target.velocity = {0_fps_, -1.0  * 13.7_fps_}; // neg dc
    target.angular_velocity = 0.0_degps_;
    target.use_dc = false;
    target.duty_cycle = 0.0;
  
    container_.drivetrain_.SetTarget(target);
  }
}

void StopTest::OnEnd(bool interrupted) {
    std::cout << "Time to brake:" << timer << std::endl;
}

bool StopTest::IsFinished() {
    return container_.drivetrain_.GetReadings().estimated_pose.velocity.magnitude().to_base() < 0.0;
}