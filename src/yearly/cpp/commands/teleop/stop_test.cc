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
  do_stop = false;
  timer = 0;

  // std::cout << "target velocity:"
  //           << container_.drivetrain_
  //                  .GetPreferenceValue_unit_type<pdcsu::units::fps_t>(
  //                      "stop_test/target_velocity")
  //                  .value()
  //           << std::endl;
}

void StopTest::Periodic() {
  auto velocity = container_.drivetrain_.GetReadings().pose.velocity;

  funkit::robot::swerve::DrivetrainTarget target{};

  if (do_stop == false) {
    target.velocity = {
        0_fps_, container_.drivetrain_
                    .GetPreferenceValue_unit_type<pdcsu::units::fps_t>(
                        "stop_test/target_velocity")};
    target.angular_velocity = 0.0_degps_;
    if (container_.drivetrain_
            .GetPreferenceValue_unit_type<pdcsu::units::fps_t>(
                "stop_test/target_velocity") <= velocity.magnitude()) {
      do_stop = true;
      start_pos = container_.drivetrain_.GetReadings().pose.position;
    }

  } else {
    timer += 0.01;
    target.velocity = {
        0_fps_, 0.05 * -13.7_fps_};  // Negative DC? Should it be 0?
    target.angular_velocity = 0.0_degps_;
    target.use_dc = false;
    target.duty_cycle = 0.0;
  }

  container_.drivetrain_.SetTarget(target);
}

void StopTest::OnEnd(bool interrupted) {
  auto end_pos = container_.drivetrain_.GetReadings().pose.position;
  auto distance = (start_pos - end_pos).magnitude();

  std::cout << "Time to brake: " << timer << std::endl;
  std::cout << "Distance: " << distance.value() << std::endl;
  std::cout
      << "End Velocity: "
      << container_.drivetrain_.GetReadings().pose.velocity.magnitude().value()
      << std::endl;

  funkit::robot::swerve::DrivetrainTarget target{};
  target.velocity = {0_fps_, 0_fps_};
  target.angular_velocity = 0.0_degps_;
  target.use_dc = false;
  target.duty_cycle = 0.0;
  container_.drivetrain_.SetTarget(target);
}

bool StopTest::IsFinished() {
  auto end_pos = container_.drivetrain_.GetReadings().pose.position;
  auto distance = (start_pos - end_pos).magnitude();

  // if (distance >= pdcsu::units::foot_t{18})
  // {
  //   std::cout << "##########Ended due to distance safety measure" <<
  //   distance.value() << std::endl;
  // }

  return do_stop &&
         ((container_.drivetrain_.GetReadings().pose.velocity[1] < 0.0_fps_));
}