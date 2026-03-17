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
  // set target to a certain velocity (make this a pref)
  do_stop = false;
  timer = 0;
  another_timer = 0;
  total_timer = 0;
  std::cout << "STARTING TEST" << std::endl;
  std::cout << "STARTING TEST" << std::endl;
  std::cout << "STARTING TEST" << std::endl;
  std::cout << "STARTING TEST" << std::endl;
  std::cout << "STARTING TEST" << std::endl;
  std::cout << "target velocity:"
            << container_.drivetrain_
                   .GetPreferenceValue_unit_type<pdcsu::units::fps_t>(
                       "stop_test/target_velocity")
                   .value()
            << std::endl;
}

void StopTest::Periodic() {
  auto velocity = container_.drivetrain_.GetReadings().pose.velocity;

  funkit::robot::swerve::DrivetrainTarget target{};
  total_timer += 0.01;
  // std::cout << "APPLIED BRAKES: curr vel " << velocity[1].value() << std::endl
  //          << " TIME NOW: " << total_timer << std::endl
  //          << "Do Stop: " << do_stop << std::endl;

  if (do_stop == false) {
    // std::cout << "Reaching target vel" << std::endl;
    target.velocity = {0_fps_, container_.drivetrain_.GetPreferenceValue_unit_type<pdcsu::units::fps_t>("stop_test/target_velocity")};
    target.angular_velocity = 0.0_degps_;
    // std::cout << "Current velocity" << velocity[1].value() << std::endl;
    if (container_.drivetrain_.GetPreferenceValue_unit_type<pdcsu::units::fps_t>("stop_test/target_velocity") <= velocity.magnitude()) {
      another_timer += 0.01;
      do_stop = true; 
      start_pos = container_.drivetrain_.GetReadings().pose.position;
    }

    container_.drivetrain_.SetTarget(target);
  } else {
    timer += 0.01;
    target.velocity = {0_fps_, 0.05 * -13.7_fps_};  // neg dc
    target.angular_velocity = 0.0_degps_;
    target.use_dc = false;
    target.duty_cycle = 0.0;

    container_.drivetrain_.SetTarget(target);
  }
  // std::cout << do_stop << std::endl;
}

void StopTest::OnEnd(bool interrupted) {
  std::cout << "Time to brake: " << timer << std::endl;
  auto end_pos = container_.drivetrain_.GetReadings().pose.position;
  auto distance = (start_pos - end_pos).magnitude();
  std::cout << "Distance: " << distance.value() << std::endl;
  std::cout << "End Velocity: " << container_.drivetrain_.GetReadings().pose.velocity.magnitude().value() << std::endl;
  funkit::robot::swerve::DrivetrainTarget target{};
  target.velocity = {0_fps_, 0_fps_};  // neg dc
  target.angular_velocity = 0.0_degps_;
  target.use_dc = false;
  target.duty_cycle = 0.0;
  container_.drivetrain_.SetTarget(target);

}

bool StopTest::IsFinished() {
  // std::cout << "Total time passed" << total_timer << std::endl;
  auto end_pos = container_.drivetrain_.GetReadings().pose.position;
  auto distance = (start_pos - end_pos).magnitude();
  // if (distance >= pdcsu::units::foot_t{18})
  // {
  //   std::cout << "##########Ended due to distance safety measure" << distance.value() << std::endl;
  // }
  return do_stop && ((container_.drivetrain_.GetReadings()
                            .pose.velocity[1] < 0.0_fps_)) ;
}