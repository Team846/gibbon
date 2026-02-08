#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "calculators/TurretPositionCalculator.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"
#include "pdcsu_units.h"

struct ICTestReadings {
  pdcsu::units::radian_t pos;
  pdcsu::units::radps_t vel;
  pdcsu::units::radian_t abs_pos;
};

struct ICTestTarget {
  pdcsu::units::radian_t pos;
  pdcsu::units::radps_t vel;
};

class ICTestSubsystem
    : public funkit::robot::GenericSubsystem<ICTestReadings, ICTestTarget> {
public:
  ICTestSubsystem();
  ~ICTestSubsystem();

  void Setup() override;

  ICTestTarget ZeroTarget() const override;

  void ZeroEncoders();

  void ZeroWithCRT();

  bool VerifyHardware() override;

private:
  ICTestReadings ReadFromHardware() override;

  void WriteToHardware(ICTestTarget target) override;

  static constexpr double teethA = 21.0;
  static constexpr double teethB = 23.0;
  static constexpr double mainTeeth = 90.0;

  ctre::phoenix6::hardware::CANcoder cancoder_1_;
  ctre::phoenix6::hardware::CANcoder cancoder_2_;

  funkit::control::HigherMotorController esc_1_;
  std::unique_ptr<pdcsu::control::ICNORPositionControl> icnor_controller_;
  std::unique_ptr<pdcsu::util::DefArmSys> arm_sys_;
  std::shared_ptr<pdcsu::control::ICNORLearner> icnor_learner_;

  TurretPositionCalculator turret_pos_calc_;
};
