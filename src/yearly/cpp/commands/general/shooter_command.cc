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
  // target.shooting_outputs_ = ShootingCalculator::GetOutputs();

  auto flip_vec =
      [](pdcsu::util::math::Vector2D vec) -> pdcsu::util::math::Vector2D {
    if (frc::DriverStation::GetAlliance() ==
        frc::DriverStation::Alliance::kBlue) {
      return pdcsu::util::math::Vector2D{
          funkit::math::FieldPoint::field_size_x - vec[0],
          funkit::math::FieldPoint::field_size_y - vec[1]};
    } else {
      return vec;
    }
  };

  if (container_.drivetrain_.GetReadings().estimated_pose.position[1] >
      200.846_in_) {
    if (container_.drivetrain_.GetReadings().estimated_pose.position[0] <
        158.32_in_) {
      ShootingCalculator::target = pdcsu::util::math::Vector2D{
          container_.GetPreferenceValue_unit_type<inch_t>("passing/left_x"),
          container_.GetPreferenceValue_unit_type<inch_t>("passing/left_y")};
    } else {
      ShootingCalculator::target = pdcsu::util::math::Vector2D{
          container_.GetPreferenceValue_unit_type<inch_t>("passing/right_x"),
          container_.GetPreferenceValue_unit_type<inch_t>("passing/right_y")};
    }
    target.tracking_state = TrackingState::kTrack;
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

  container_.scorer_ss_.SetTarget(target);
}

void ShooterCommand::OnEnd(bool interrupted) {}

bool ShooterCommand::IsFinished() { return false; }