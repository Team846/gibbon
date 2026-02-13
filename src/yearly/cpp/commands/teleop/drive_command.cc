#include "commands/teleop/drive_command.h"

#include <utility>

#include "calculators/ShootingCalculator.h"
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

  Graph("target_velocity_x", target.velocity[0]);
  Graph("target_velocity_y", target.velocity[1]);

  target.angular_velocity = degps_t{rotation * max_omega.value()};

  /* For shooting while in motion */
  if (ci_readings_.prepare_shot) {
    ShootingCalculatorOutputs shooting_outputs =
        ShootingCalculator::GetOutputs();
    if (shooting_outputs.is_valid) {
      target.angular_velocity = container_.drivetrain_.ApplyBearingPID(
          shooting_outputs.aim_angle,
          shooting_outputs.vel_aim_compensation);
    }
  }

  /* Do not remove the following code */

  bool isBlue = (frc::DriverStation::GetAlliance() ==
                 frc::DriverStation::Alliance::kBlue);

  if (isBlue) target.velocity = target.velocity.rotate(degree_t{180});

  container_.drivetrain_.SetTarget({target});

  // TODO remove
  auto err = (pdcsu::util::math::Vector2D{144.85_in_, 158.34_in_} -
              container_.drivetrain_.GetReadings().estimated_pose.position);
  auto rpos = pdcsu::util::math::Vector2D{3.0_in_, 0.0_in_}.rotate(
      container_.drivetrain_.GetReadings().pose.bearing, true);
  degree_t tbb =
      container_.drivetrain_.GetReadings().pose.bearing - err.angle(true);
  Graph("errangle", err.angle(true));
  icpostarget = icposz + tbb;
  if (icpostarget > 270_deg_) {
    icposz -= 360_deg_;
    icpostarget = icposz + tbb;
  } else if (icpostarget < -270_deg_) {
    icposz += 360_deg_;
    icpostarget = icposz + tbb;
  }
  radps_t icposvel = container_.drivetrain_.GetReadings().yaw_rate * 1.5;
  ICTestTarget ictarget{};
  ictarget.pos = icpostarget;
  ictarget.vel = icposvel;

  container_.ictest_.SetTarget(ictarget);
}

void DriveCommand::OnEnd(bool interrupted) {}

bool DriveCommand::IsFinished() { return false; }