#include "commands/teleop/drive_command.h"

#include <utility>

#include "calculators/AntiTippingCalculator.h"
#include "funkit/math/fieldpoints.h"

DriveCommand::DriveCommand(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, DriveCommand>{
          container, "drive_command"} {
  AddRequirements({&container_.drivetrain_});
}

void DriveCommand::OnInit() {}

void DriveCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};

  funkit::robot::swerve::DrivetrainTarget target{};

  container_.drivetrain_.SetTarget({target});

  double translate_x = funkit::math::HorizontalDeadband(
      ci_readings_.translate_x,
      container_.control_input_.GetPreferenceValue_double(
          "translation_deadband"),
      1,
      container_.control_input_.GetPreferenceValue_int("translation_exponent"),
      1);

  double translate_y = funkit::math::HorizontalDeadband(
      ci_readings_.translate_y,
      container_.control_input_.GetPreferenceValue_double(
          "translation_deadband"),
      1,
      container_.control_input_.GetPreferenceValue_int("translation_exponent"),
      1);

  double rotation = funkit::math::HorizontalDeadband(ci_readings_.rotation,
      container_.control_input_.GetPreferenceValue_double("rotation_deadband"),
      1, container_.control_input_.GetPreferenceValue_int("rotation_exponent"),
      1);

  fps_t max_speed =
      container_.drivetrain_.GetPreferenceValue_unit_type<fps_t>("max_speed");
  degps_t max_omega =
      container_.drivetrain_.GetPreferenceValue_unit_type<degps_t>("max_omega");

  target.velocity = {fps_t{translate_x * max_speed.value()},
      fps_t{translate_y * max_speed.value()}};

  auto delta_dir =
      (target.velocity - container_.drivetrain_.GetReadings().pose.velocity);

  Graph("delta_dir_x", delta_dir[0]);
  Graph("delta_dir_y", delta_dir[1]);

  // auto accel_limited = AntiTippingCalculator::LimitAcceleration(
  //     delta_dir, container_.drivetrain_.GetReadings().pose.bearing);

  // Graph("limited_accel_x", accel_limited[0]);
  // Graph("limited_accel_y", accel_limited[1]);

  // target.velocity[0] =
  //     1_fps * rampRateLimiter_x_.limit(target.velocity[0].to<double>(),
  //                 accel_limited[0].to<double>());
  // target.velocity[1] =
  //     1_fps * rampRateLimiter_y_.limit(target.velocity[1].to<double>(),
  //                 accel_limited[1].to<double>());

  target.angular_velocity = degps_t{rotation * max_omega.value()};

  bool isBlue = (frc::DriverStation::GetAlliance() ==
                 frc::DriverStation::Alliance::kBlue);

  if (isBlue) target.velocity = target.velocity.rotate(degree_t{180});

  container_.drivetrain_.SetTarget({target});
}

void DriveCommand::OnEnd(bool interrupted) {}

bool DriveCommand::IsFinished() { return false; }