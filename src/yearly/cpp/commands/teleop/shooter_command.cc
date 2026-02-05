#include "commands/teleop/shooter_command.h"

#include <utility>

#include "calculators/ShootingCalculator.h"
#include "funkit/math/fieldpoints.h"

ShooterCommand::ShooterCommand(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, ShooterCommand>{
          container, "shooter_command"} {
  AddRequirements({&container_.shooter_});
}

void ShooterCommand::OnInit() {}

void ShooterCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};

  ShooterTarget target{};

  if (ci_readings_.prepare_shot) {
    ShootingCalculatorOutputs shooting_outputs =
        ShootingCalculator::GetOutputs();
    if (shooting_outputs.is_valid) {
      target.target_vel = shooting_outputs.shooter_vel;
    }
  }

  container_.shooter_.SetTarget(target);
}

void ShooterCommand::OnEnd(bool interrupted) {}

bool ShooterCommand::IsFinished() { return false; }