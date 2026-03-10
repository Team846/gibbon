#include "commands/general/auto_intake_command.h"

#include "calculators/ShootingCalculator.h"
#include "funkit/math/fieldpoints.h"

frc2::InstantCommand AutoIntakeCommand(
    RobotContainer &container, HoptakeState target) {
  return frc2::InstantCommand{[&container, target] {
    container.hoptake_ss_.SetDefaultCommand(frc2::FunctionalCommand{[] {},
        [&container, target] {
          HoptakeSSTarget t{};
          t.target_state = target;
          t.drivetrain_vel = container.drivetrain_.GetReadings()
                                 .estimated_pose.velocity.magnitude();
          container.hoptake_ss_.SetTarget(t);
        },
        [](bool) {}, [] { return false; }, {&container.hoptake_ss_}});
  }};
}