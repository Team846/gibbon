#pragma once

#include <deque>
#include <memory>

#include "calculators/TurretPositionCalculator.h"
#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/robot/swerve/drivetrain.h"
#include "funkit/wpilib/time.h"
#include "subsystems/hardware/hoptake/intake.h"
#include "subsystems/hardware/hoptake/pivot.h"

enum class HoptakeState { kIdle, kIntake, kAgitate, kEvac };

struct HoptakeSSReadings {};

struct HoptakeSSTarget {
  HoptakeState target_state;
  fps_t drivetrain_vel;
};

class HoptakeSuperstructure
    : public funkit::robot::GenericSubsystem<HoptakeSSReadings,
          HoptakeSSTarget> {
public:
  HoptakeSuperstructure();
  ~HoptakeSuperstructure();

  void Setup() override;

  HoptakeSSTarget ZeroTarget() const override;

  IntakeSubsystem intake;
  PivotSubsystem pivot;

  bool HasReachedShooter(degps_t vel);
  bool HasReachedHood(degree_t pos);
  bool HasReachedTurret(degree_t pos);

  void AdjustTurret(bool cw);
  void AdjustHood(bool up);
  void ClearAdjustments();

  bool VerifyHardware() override;

  void ZeroEncoders();

private:
  HoptakeSSReadings ReadFromHardware() override;

  void WriteToHardware(HoptakeSSTarget target) override;

  bool agitate_reached_ = false;
};
