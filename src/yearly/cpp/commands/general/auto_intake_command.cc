#include "commands/general/auto_intake_command.h"

#include "calculators/ShootingCalculator.h"
#include "funkit/math/fieldpoints.h"

AutoIntakeCommand::AutoIntakeCommand(
    RobotContainer &container, HoptakeState target)
    : funkit::robot::GenericCommand<RobotContainer,
          AutoIntakeCommand>{container, "auto_intake_command"},
      target_{target} {
  AddRequirements({&container_.hoptake_ss_});
}

void AutoIntakeCommand::OnInit() {}

void AutoIntakeCommand::Periodic() {
  HoptakeSSTarget target{};
  target.target_state = target_;
  target.drivetrain_vel =
      container_.drivetrain_.GetReadings().estimated_pose.velocity.magnitude();
  container_.hoptake_ss_.SetTarget(target);
}

void AutoIntakeCommand::OnEnd(bool interrupted) {}

bool AutoIntakeCommand::IsFinished() { return true; }