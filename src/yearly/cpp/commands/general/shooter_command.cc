#include "commands/general/shooter_command.h"

#include <frc/DriverStation.h>

#include <utility>

#include "calculators/ShootingCalculator.h"
#include "funkit/math/fieldpoints.h"

ShooterCommand::ShooterCommand(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, ShooterCommand>{
          container, "shooter_command"} {
  AddRequirements({&container_.scorer_ss_});
}

void ShooterCommand::OnInit() {}

void ShooterCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};
  ScorerSSTarget target{};
  ShootingCalculatorOutputs shooting_outputs;
  bool mirror_ =
      frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;

  if (container_.drivetrain_.GetReadings().estimated_pose.position[1] >
          195.846_in_ &&
      container_.drivetrain_.GetReadings().estimated_pose.position[1] <
          450.61_in_) {
    if (!mirror_ &&
        container_.drivetrain_.GetReadings().estimated_pose.position[0] <
            158.32_in_) {
      ShootingCalculator::target =
          funkit::math::FieldPoint{
              {container_.GetPreferenceValue_unit_type<inch_t>(
                   "passing/left_x"),
                  container_.GetPreferenceValue_unit_type<inch_t>(
                      "passing/left_y")},
              0_deg_, 0_fps_}
              .mirror(mirror_)
              .point;
    } else {
      ShootingCalculator::target =
          funkit::math::FieldPoint{
              {container_.GetPreferenceValue_unit_type<inch_t>(
                   "passing/right_x"),
                  container_.GetPreferenceValue_unit_type<inch_t>(
                      "passing/right_y")},
              0_deg_, 0_fps_}
              .mirror(mirror_)
              .point;
    }
  } else {
    ShootingCalculator::target =
        funkit::math::FieldPoint{{158.845_in_, 182.11_in_}, 0_deg_, 0_fps_}
            .mirror(mirror_)
            .point;
  }

  if (ci_readings_.turret_trim_cw) {
    container_.scorer_ss_.AdjustTurret(true);
  } else if (ci_readings_.turret_trim_ccw) {
    container_.scorer_ss_.AdjustTurret(false);
  }

  if (ci_readings_.hood_trim_cw) {
    container_.scorer_ss_.AdjustHood(true);
  } else if (ci_readings_.hood_trim_ccw) {
    container_.scorer_ss_.AdjustHood(false);
  }

  if (ci_readings_.turret_no_spin) {
    target.tracking_state = TrackingState::kLockTurret;
  } else if (ci_readings_.point_blank_shot) {
    target.tracking_state = TrackingState::kPointBlank;
  } else {
    target.tracking_state = TrackingState::kTrack;
  }

  if (ci_readings_.rev_dye_rotor) { target.reverse_rotor = true; }

  shooting_outputs = ShootingCalculator::GetOutputs();

  target.shooter_target = shooting_outputs.shooter_vel;
  target.turret_target = {
      shooting_outputs.aim_angle, shooting_outputs.vel_aim_compensation};
  target.shoot = shooting_outputs.is_valid && !ci_readings_.override_autoshoot;

  container_.scorer_ss_.SetTarget(target);
}

void ShooterCommand::OnEnd(bool interrupted) {}

bool ShooterCommand::IsFinished() { return false; }