#include "funkit/robot/swerve/wait_until_close.h"

namespace funkit::robot::swerve {

WaitUntilClose::WaitUntilClose(
    DrivetrainSubsystem* drivetrain, funkit::math::FieldPoint target)
    : Loggable("WaitUntilClose"), drivetrain_(drivetrain), target_(target) {
  SetName("WaitUntilClose");
  AddRequirements({});
}

void WaitUntilClose::Initialize() { Log("WaitUntilClose initialized"); }

void WaitUntilClose::Execute() {}

void WaitUntilClose::End(bool interrupted) {}

bool WaitUntilClose::IsFinished() {
  return (drivetrain_->GetReadings().estimated_pose.position - target_.point)
             .magnitude() < pdcsu::units::inch_t{0.5};
}

}  // namespace funkit::robot::swerve