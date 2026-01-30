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

  IntakeTarget target{IntakeState::kIdle};

  if (ci_readings_.shoot && container_.shooter_.GetReadings().is_spun_up)
    target.state = IntakeState::kIntake;

  target.realintake = ci_readings_.intake;
  container_.intake_.SetTarget(target);
}

void IntakeCommand::OnEnd(bool interrupted) {}

bool IntakeCommand::IsFinished() { return false; }