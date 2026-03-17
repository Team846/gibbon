#include "control_triggers.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>

#include "commands/teleop/climb_align_command.h"
#include "commands/teleop/ramp_accel_command.h"
#include "commands/teleop/speed_test_command.h"
#include "commands/teleop/stop_test.h"

void ControlTriggerInitializer::InitTeleopTriggers(RobotContainer& container) {
  frc2::Trigger drivetrain_zero_bearing_trigger{[&] {
    return container.control_input_.GetReadings().zero_bearing;
  }};
  drivetrain_zero_bearing_trigger.WhileTrue(frc2::InstantCommand([&] {
    container.drivetrain_.ZeroBearing();
  }).ToPtr());

  frc2::Trigger stop_test_trigger{[&] {
    return container.control_input_.GetReadings().stop_test;
  }};
  stop_test_trigger.WhileTrue(StopTest{container}.ToPtr());

  frc2::Trigger speed_test_trigger{[&] {
    return container.control_input_.GetReadings().speed_test;
  }};
  speed_test_trigger.WhileTrue(SpeedTestCommand{container}.ToPtr());

  frc2::Trigger ramp_accel_trigger{[&] {
    return container.control_input_.GetReadings().ramp_accel_test;
  }};
  ramp_accel_trigger.WhileTrue(RampAccelCommand{container}.ToPtr());
}
