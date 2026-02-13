#include "commands/general/shooter_command.h"

#include <utility>

#include "calculators/ShootingCalculator.h"
#include "funkit/math/fieldpoints.h"

ShooterCommand::ShooterCommand(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, ShooterCommand>{
          container, "shooter_command"} {
  AddRequirements({});
}

void ShooterCommand::OnInit() {}

void ShooterCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};

  ScorerSSTarget target{};

  ShootingCalculatorOutputs shooting_outputs = ShootingCalculator::GetOutputs();

  target.shooting_outputs_ = shooting_outputs;

  if (ci_readings_.override_autoshoot) {
    target.override_state_ = ScorerOverrides::kOverrideAutoShoot;
  } else if (ci_readings_.force_shoot) {
    target.override_state_ = ScorerOverrides::kForceShoot;
  }

  if (container_.drivetrain_.GetReadings().estimated_pose.position[1] >
      200.846_in_) {
    if (container_.drivetrain_.GetReadings().estimated_pose.position[0] <
        158.32_in_) {
      target.state_ = ScorerState::kPassingLeft;
    } else {
      target.state_ = ScorerState::kPassingRight;
    }
  }

  if (ci_readings_.turret_trim > 0.5) {
    container_.scorer_ss_.AdjustTurret(false);
  } else if (ci_readings_.turret_trim < -0.5) {
    container_.scorer_ss_.AdjustTurret(true);
  }

  if (ci_readings_.hood_trim > 0.5) {
    container_.scorer_ss_.AdjustHood(true);
  } else if (ci_readings_.hood_trim < -0.5) {
    container_.scorer_ss_.AdjustHood(false);
  }

  container_.scorer_ss_.SetTarget(target);
}

void ShooterCommand::OnEnd(bool interrupted) {}

bool ShooterCommand::IsFinished() { return false; }