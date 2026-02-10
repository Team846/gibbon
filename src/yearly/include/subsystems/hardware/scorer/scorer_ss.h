// #pragma once

// #include <deque>
// #include <memory>

// #include "calculators/TurretPositionCalculator.h"
// #include "funkit/control/HigherMotorController.h"
// #include "funkit/robot/GenericRobot.h"
// #include "funkit/robot/GenericSubsystem.h"
// #include "funkit/robot/swerve/drivetrain.h"
// #include "funkit/wpilib/time.h"
// #include "subsystems/hardware/scorer/hood.h"
// #include "subsystems/hardware/scorer/shooter.h"
// #include "subsystems/hardware/scorer/turret.h"

// enum class ScorerState { kFollowHub, kPassing, kPointBlank };

// enum class ScorerOverrides { kNothing, kForceShoot, kOverrideAutoShoot };

// struct ScorerSSReadings {
//   ScorerState state;
// };

// struct ScorerSSTarget {
//   funkit::robot::swerve::odometry::SwervePose pose;
//   ScorerOverrides override_state = ScorerOverrides::kNothing;
// };

// class ScorerSuperstructure
//     : public funkit::robot::GenericSubsystem<ScorerSSReadings,
//     ScorerSSTarget> {
// public:
//   ScorerSuperstructure();
//   ~ScorerSuperstructure();

//   void Setup() override;

//   ScorerSSTarget ZeroTarget() const override;

//   TurretSubsystem turret;
//   HoodSubsystem hood;
//   ShooterSubsystem shooter;

//   bool HasReachedShooter(degps_t vel);
//   bool HasReachedHood(degree_t pos);
//   bool HasReachedTurret(degree_t pos);

//   void AdjustTurret(bool cw);
//   void AdjustHood(bool up);
//   void ClearAdjustments();

//   bool VerifyHardware() override;

//   void ZeroEncoders();

// private:
//   ScorerSSReadings ReadFromHardware() override;

//   degree_t turret_adjustment_ = 0_deg_;
//   degree_t hood_adjustment_ = 0_deg_;

//   void WriteToHardware(ScorerSSTarget target) override;
// };
