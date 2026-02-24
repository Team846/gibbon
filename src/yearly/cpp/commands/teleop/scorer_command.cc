#include "commands/teleop/scorer_command.h"

#include <frc/DriverStation.h>

#include <utility>

#include "calculators/ShootingCalculator.h"
#include "funkit/math/fieldpoints.h"

ScorerCommand::ScorerCommand(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, ScorerCommand>{
          container, "shooter_command"} {
  AddRequirements({&container_.scorer_ss_});
}

void ScorerCommand::OnInit() {}

void ScorerCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};
  ScorerSSTarget target{};
  ShootingCalculatorOutputs shooting_outputs;
  bool mirror_ =
      frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;

  if (ci_readings_.pass_mode) {
    pdcsu::util::math::Vector2D pass_point{-1000_in_, -1000_in_};
    if (container_.drivetrain_.GetReadings().estimated_pose.position[0] <
        138.32_in_) {
      pass_point = {container_.scorer_ss_.GetPreferenceValue_unit_type<inch_t>(
                        "passing/left_x"),
          container_.scorer_ss_.GetPreferenceValue_unit_type<inch_t>(
              "passing/left_y")};
      Graph("left", true);
    } else if (container_.drivetrain_.GetReadings().estimated_pose.position[0] >
               178.32_in_) {
      pass_point = {container_.scorer_ss_.GetPreferenceValue_unit_type<inch_t>(
                        "passing/right_x"),
          container_.scorer_ss_.GetPreferenceValue_unit_type<inch_t>(
              "passing/right_y")};
      Graph("right", true);
    } else {
      Graph("left", false);
      Graph("right", false);
    }
    funkit::math::FieldPoint field_point = {pass_point, 0_deg_, 0_fps_};
    field_point = field_point.mirrorOnlyY(mirror_);
    ShootingCalculator::target = field_point.point;
    Graph("pass", true);
    Graph("shoot", false);
  } else {
    ShootingCalculator::target =
        funkit::math::FieldPoint{{158.845_in_, 182.11_in_}, 0_deg_, 0_fps_}
            .mirror(mirror_)
            .point;
    Graph("shoot", true);
    Graph("pass", false);
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

  if (!ci_readings_.pass_mode) {
    if (!mirror_ &&
        container_.drivetrain_.GetReadings().estimated_pose.position[1] >
            141.61_in_) {
      shooting_outputs.is_valid = false;
    }
    if (mirror_ &&
        container_.drivetrain_.GetReadings().estimated_pose.position[1] <
            509.61_in_) {
      shooting_outputs.is_valid = false;
    }
  }

  target.shooter_target = shooting_outputs.shooter_vel;
  target.hood_target = {
      shooting_outputs.shot_angle, shooting_outputs.shot_angle_vel};
  target.turret_target = {shooting_outputs.aim_angle -
                              container_.drivetrain_.GetReadings().pose.bearing,
      shooting_outputs.vel_aim_compensation -
          container_.drivetrain_.GetReadings().yaw_rate};
  target.shoot =
      ci_readings_.point_blank_shot ||
      (shooting_outputs.is_valid && !ci_readings_.override_autoshoot &&
          container_.scorer_ss_.GetReadings().will_make_shot &&
          !frc::DriverStation::IsTest()) ||
      ci_readings_.force_shoot;

  if (!target.shoot) {
    container_.drivetrain_.SetFieldObjectPose(
        "shoot_point", {-1000_in_, -1000_in_}, 0.0_deg_);
    container_.drivetrain_.SetFieldObjectPose(
        "pass_point", {-1000_in_, -1000_in_}, 0.0_deg_);
    container_.drivetrain_.SetFieldTrajectory(
        {-1000_in_, -1000_in_}, {-1100_in_, -1100_in_});
  } else {
    container_.drivetrain_.SetFieldTrajectory(
        shooting_outputs.start_traj, shooting_outputs.term_traj);

    if (ci_readings_.pass_mode) {
      container_.drivetrain_.SetFieldObjectPose(
          "shoot_point", {-1000_in_, -1000_in_}, 0.0_deg_);
      container_.drivetrain_.SetFieldObjectPose(
          "pass_point", ShootingCalculator::target, 0.0_deg_);
    } else {
      container_.drivetrain_.SetFieldObjectPose(
          "shoot_point", ShootingCalculator::target, 0.0_deg_);
      container_.drivetrain_.SetFieldObjectPose(
          "pass_point", {-1000_in_, -1000_in_}, 0.0_deg_);
    }
  }

  container_.scorer_ss_.SetTarget(target);
}

void ScorerCommand::OnEnd(bool interrupted) {}

bool ScorerCommand::IsFinished() { return false; }