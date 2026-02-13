#include "commands/general/intake_command.h"

#include <utility>

#include "calculators/ShootingCalculator.h"
#include "funkit/math/fieldpoints.h"

IntakeCommand::IntakeCommand(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, IntakeCommand>{
          container, "intake_command"} {
  AddRequirements({&container_.hoptake_ss_});
}

void IntakeCommand::OnInit() {}

void IntakeCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};

  HoptakeSSTarget target;
  target.intake_speed = ci_readings_.intake_speed;

  if (ci_readings_.agitate) {
    target.override_state = HoptakeOverrides::kAgitate;
  } else if (ci_readings_.evac_storage) {
    target.override_state = HoptakeOverrides::kEvac;
  }

  container_.hoptake_ss_.SetTarget(target);
}

void IntakeCommand::OnEnd(bool interrupted) {}

bool IntakeCommand::IsFinished() { return false; }