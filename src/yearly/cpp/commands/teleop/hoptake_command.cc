#include "commands/teleop/hoptake_command.h"

#include <utility>

#include "calculators/ShootingCalculator.h"
#include "funkit/math/fieldpoints.h"

HoptakeCommand::HoptakeCommand(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, HoptakeCommand>{
          container, "hoptake_command"} {
  AddRequirements({&container_.hoptake_ss_});
}

void HoptakeCommand::OnInit() {}

void HoptakeCommand::Periodic() {
  ControlInputReadings ci_readings_{container_.control_input_.GetReadings()};
  HoptakeSSTarget target;

  if (ci_readings_.intake) {
    target.target_state = HoptakeState::kIntake;
  } else if (ci_readings_.agitate) {
    target.target_state = HoptakeState::kAgitate;
  } else if (ci_readings_.evac_storage) {
    target.target_state = HoptakeState::kEvac;
  } else {
    target.target_state = HoptakeState::kIdle;
  }

  target.drivetrain_vel =
      container_.drivetrain_.GetReadings().estimated_pose.velocity.magnitude();

  container_.hoptake_ss_.SetTarget(target);
}

void HoptakeCommand::OnEnd(bool interrupted) {}

bool HoptakeCommand::IsFinished() { return false; }