#include "commands/teleop/stop_test.h"
#include <iostream>

StopTest::StopTest(RobotContainer &container)
    : funkit::robot::GenericCommand<RobotContainer, StopTest>{
          container, "stop_test"} {
  AddRequirements({&container_.drivetrain_});
}

void StopTest::OnInit() {

}

void StopTest::Periodic() {

}

void StopTest::OnEnd(bool interrupted) {
    
}

bool StopTest::IsFinished() { return false; }