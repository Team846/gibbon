#include "commands/teleop/intake_command.h"

#include <utility>

#include "calculators/ShootingCalculator.h"
#include "funkit/math/fieldpoints.h"

IntakeCommand::IntakeCommand(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, IntakeCommand>{
          container, "intake_command"} {
  AddRequirements({&container_.intake_});
}

void IntakeCommand::OnInit() {}

void IntakeCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};

  IntakeTarget target{};

  if (ci_readings_.shoot)
    target.state = IntakeState::kIntake;
  else
    target.state = IntakeState::kIdle;

  container_.intake_.SetTarget(target);
}

void IntakeCommand::OnEnd(bool interrupted) {}

bool IntakeCommand::IsFinished() { return false; }