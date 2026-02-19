#include "commands/general/auto_scorer_command.h"

#include "calculators/ShootingCalculator.h"
#include "funkit/math/fieldpoints.h"

frc2::InstantCommand AutoScorerCommand(
    RobotContainer &container, bool enable_shooting) {
  return frc2::InstantCommand{[&container, enable_shooting] {
    container.scorer_ss_.SetDefaultCommand(frc2::FunctionalCommand{[] {},
        [&container, enable_shooting] {
          ShootingCalculatorOutputs shooting_outputs;
          ScorerSSTarget scorer_target{};
          bool mirror = frc::DriverStation::GetAlliance() ==
                        frc::DriverStation::Alliance::kBlue;
          ShootingCalculator::target =
              funkit::math::FieldPoint{
                  {158.845_in_, 182.11_in_}, 0_deg_, 0_fps_}
                  .mirror(mirror)
                  .point;

          container.drivetrain_.SetFieldObjectPose(
              "shoot_point", ShootingCalculator::target, 0.0_deg_);
          container.drivetrain_.SetFieldObjectPose(
              "pass_point", {-1000_in_, -1000_in_}, 0.0_deg_);

          shooting_outputs = ShootingCalculator::GetOutputs();

          auto drivetrain_readings = container.drivetrain_.GetReadings();

          if (shooting_outputs.is_valid) {
            container.drivetrain_.SetFieldTrajectory(
                shooting_outputs.start_traj, shooting_outputs.term_traj);
          } else {
            container.drivetrain_.SetFieldObjectPose(
                "shoot_point", {-1000_in_, -1000_in_}, 0.0_deg_);
            container.drivetrain_.SetFieldTrajectory(
                {-1000_in_, -1000_in_}, {-1100_in_, -1100_in_});
          }

          scorer_target.shooter_target = shooting_outputs.shooter_vel;
          scorer_target.turret_target = {
              shooting_outputs.aim_angle - drivetrain_readings.pose.bearing,
              shooting_outputs.vel_aim_compensation -
                  drivetrain_readings.yaw_rate};
          scorer_target.shoot = enable_shooting && shooting_outputs.is_valid;
          scorer_target.tracking_state = TrackingState::kTrack;

          container.scorer_ss_.SetTarget(scorer_target);
        },
        [](bool) {}, [] { return false; }, {&container.scorer_ss_}});
  }};
}
