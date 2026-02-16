#pragma once

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "subsystems/hardware/climb/telescope.h"
#include "subsystems/hardware/climb/climber.h"

enum class ClimbState { kIdle, kIntake, kAgitate, kEvac };

struct ClimbSSReadings {};

struct ClimbSSTarget {
  ClimbState target_state;
  fps_t drivetrain_vel;
};

class ClimbSuperstructure
    : public funkit::robot::GenericSubsystem<ClimbSSReadings,
          ClimbSSTarget> {
public:
  ClimbSuperstructure();
  ~ClimbSuperstructure();

  void Setup() override;

  ClimbSSTarget ZeroTarget() const override;

  ClimberSubsystem climber;
  TelescopeSubsystem telescope;

  bool VerifyHardware() override;

  void ZeroEncoders();

private:
  ClimbSSReadings ReadFromHardware() override;

  void WriteToHardware(ClimbSSTarget target) override;
};
