#include "commands/teleop/speed_test_command.h"

#include <iostream>

SpeedTestCommand::SpeedTestCommand(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, SpeedTestCommand>{
          container, "speed_test_command"} {
  AddRequirements({&container_.drivetrain_});
}

void SpeedTestCommand::OnInit() {}

void SpeedTestCommand::Periodic() {
  funkit::robot::swerve::DrivetrainTarget target{};
  target.velocity = {0_fps_,
      container_.drivetrain_.GetPreferenceValue_unit_type<pdcsu::units::fps_t>(
          "max_speed")};  // hardcoded max speed?
  target.angular_velocity = pdcsu::units::degps_t{0};

  container_.drivetrain_.SetTarget(target);
}

void SpeedTestCommand::OnEnd(bool interrupted) {
  funkit::robot::swerve::DrivetrainTarget target{};
  target.velocity = {0_fps_, 0_fps_};
  target.angular_velocity = pdcsu::units::degps_t{0};
  container_.drivetrain_.SetTarget(target);
}

bool SpeedTestCommand::IsFinished() {
  std::cout << container_.drivetrain_.GetReadings().pose.velocity.magnitude().value() << std::endl;
  return (
      container_.drivetrain_.GetReadings().pose.velocity.magnitude() >=
      container_.drivetrain_.GetPreferenceValue_unit_type<pdcsu::units::fps_t>(
          "speed_test_command/max_velocity"));  // hardcoded max speed?
}
