#include "commands/teleop/gpd_command.h"

DriveGPDCommand::DriveGPDCommand(RobotContainer& container)
    : funkit::robot::GenericCommand<RobotContainer, DriveGPDCommand>{container,
          "drive_gpd_command"},
      container_{container} {
  AddRequirements({&container_.drivetrain_});
}

void DriveGPDCommand::OnInit() {}

void DriveGPDCommand::Periodic() {
  auto gpd_readings = container_.GPD_.GetReadings();
  if (!gpd_readings.has_target) {
    container_.drivetrain_.SetTargetZero();
    return;
  }

  auto dt_readings = container_.drivetrain_.GetReadings();
  auto vel = dt_readings.estimated_pose.velocity;
  if (vel.magnitude() < pdcsu::units::fps_t{0.1}) {
    container_.drivetrain_.SetTargetZero();
    return;
  }

  funkit::robot::swerve::DrivetrainTarget target;
  target.velocity = vel;

  double k = container_.GPD_.GetPreferenceValue_double("kS");
  double x_error =
      (gpd_readings.optimal_pos[1] - dt_readings.estimated_pose.position[1])
          .value();
  double vel_y = vel[0].value();
  target.velocity = {target.velocity[0],
      pdcsu::units::fps_t{target.velocity[1].value() + k * x_error * vel_y}};

  container_.drivetrain_.SetTarget(target);
}

void DriveGPDCommand::OnEnd(bool interrupted) {}

bool DriveGPDCommand::IsFinished() { return false; }
