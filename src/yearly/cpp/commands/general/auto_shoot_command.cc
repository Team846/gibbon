#include "commands/general/auto_shoot_command.h"

#include "calculators/ShootingCalculator.h"
#include "funkit/math/fieldpoints.h"

AutoShootCommand::AutoShootCommand(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, AutoShootCommand>{
          container, "auto_shoot_command"} {
  AddRequirements({&container_.scorer_ss_});
}

void AutoShootCommand::OnInit() {}

void AutoShootCommand::Periodic() {
  ShootingCalculatorOutputs shooting_outputs;
  ScorerSSTarget scorer_target{};
  bool mirror_ =
      frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
  ShootingCalculator::target =
      funkit::math::FieldPoint{{158.845_in_, 182.11_in_}, 0_deg_, 0_fps_}
          .mirror(mirror_)
          .point;
  shooting_outputs = ShootingCalculator::GetOutputs();

  auto drivetrain_readings = container_.drivetrain_.GetReadings();

  scorer_target.shooter_target = shooting_outputs.shooter_vel;
  scorer_target.turret_target = {
      shooting_outputs.aim_angle - drivetrain_readings.pose.bearing,
      shooting_outputs.vel_aim_compensation - drivetrain_readings.yaw_rate};
  scorer_target.shoot = shooting_outputs.is_valid;
  scorer_target.tracking_state = TrackingState::kTrack;

  container_.scorer_ss_.SetTarget(scorer_target);
}

void AutoShootCommand::OnEnd(bool interrupted) {}

bool AutoShootCommand::IsFinished() { return false; }