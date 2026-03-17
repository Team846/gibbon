#include "commands/teleop/ramp_accel_command.h"

#include <iostream>

RampAccelCommand::RampAccelCommand(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, RampAccelCommand>{
          container, "ramp_accel_command"} {
  AddRequirements({&container_.drivetrain_});
}

void RampAccelCommand::OnInit() {
  start_accel = false;
  initial_timer = 0;
  accel_timer = 0;
}

void RampAccelCommand::Periodic() {
  funkit::robot::swerve::DrivetrainTarget target{};
  target.velocity = {0_fps_, 0_fps_};
  if (!start_accel) {
    target.velocity = {0_fps_,
        container_.drivetrain_
            .GetPreferenceValue_unit_type<pdcsu::units::fps_t>(
                "ramp_accel_command/initial_velocity")};  // Should we hardcode
                                                          // to 1 fps?
    if (initial_timer >= 1) { start_accel = true; }
    initial_timer += 0.01;
  } else {
    target.velocity = {0_fps_,
        container_.drivetrain_
            .GetPreferenceValue_unit_type<pdcsu::units::fps_t>(
                "ramp_accel_command/final_velocity")};  // Should we hardcode to
                                                        // 3 fps?
    accel_timer += 0.01;
  }
  target.angular_velocity = pdcsu::units::degps_t{0};

  container_.drivetrain_.SetTarget(target);
}

void RampAccelCommand::OnEnd(bool interrupted) {
  funkit::robot::swerve::DrivetrainTarget target{};
  target.velocity = {0_fps_, 0_fps_};
  target.angular_velocity = pdcsu::units::degps_t{0};
  container_.drivetrain_.SetTarget(target);

  std::cout << "Time to accelerate from "
            << container_.drivetrain_
                   .GetPreferenceValue_unit_type<pdcsu::units::fps_t>(
                       "ramp_accel_command/initial_velocity")
                   .value()
            << " fps to "
            << container_.drivetrain_
                   .GetPreferenceValue_unit_type<pdcsu::units::fps_t>(
                       "ramp_accel_command/final_velocity")
                   .value()
            << " fps: " << accel_timer << std::endl;
  // Check if acceleration is smooth through velocity graph
}

bool RampAccelCommand::IsFinished() {
  return (
      container_.drivetrain_.GetPreferenceValue_unit_type<pdcsu::units::fps_t>(
          "ramp_accel_command/final_velocity") <
      container_.drivetrain_.GetReadings().pose.velocity.magnitude());
}