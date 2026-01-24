#pragma once

#include <deque>
#include <memory>

#include "calculators/TurretPositionCalculator.h"
#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

struct TurretTestReadings {};

struct TurretTestTarget {
  pdcsu::units::rotation_t pos;
};

class TurretTestSubsystem
    : public funkit::robot::GenericSubsystem<TurretTestReadings,
          TurretTestTarget> {
public:
  TurretTestSubsystem();
  ~TurretTestSubsystem();

  void Setup() override;

  TurretTestTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroEncoders();

private:
  TurretTestReadings ReadFromHardware() override;

  static constexpr double teethA = 20.0;
  static constexpr double teethB = 27.0;
  static constexpr double mainTeeth = 90.0;

  void WriteToHardware(TurretTestTarget target) override;

  funkit::control::HigherMotorController esc_1_;
  funkit::control::HigherMotorController esc_2_;
  std::unique_ptr<pdcsu::util::DefArmSys> arm_sys_;

  TurretPositionCalculator turret_position_calculator_;
};
