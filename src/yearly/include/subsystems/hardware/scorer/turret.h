#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <deque>
#include <memory>

#include "calculators/TurretPositionCalculator.h"
#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

struct TurretReadings {
  degree_t pos_;
  degps_t vel_;
  bool in_position_;
};

struct TurretTarget {
  degree_t pos_;
  degps_t vel_;
};

class TurretSubsystem
    : public funkit::robot::GenericSubsystem<TurretReadings, TurretTarget> {
public:
  TurretSubsystem();
  ~TurretSubsystem();

  void Setup() override;

  TurretTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroEncoders();
  void ZeroWithCRT();

private:
  TurretReadings ReadFromHardware() override;
  void WriteToHardware(TurretTarget target) override;

  static constexpr double teethA = 21.0;
  static constexpr double teethB = 23.0;
  static constexpr double mainTeeth = 90.0;

  funkit::control::HigherMotorController esc_;

  ctre::phoenix6::hardware::CANcoder cancoder_1_;
  ctre::phoenix6::hardware::CANcoder cancoder_2_;

  std::unique_ptr<pdcsu::control::ICNORPositionControl> icnor_controller_;
  std::unique_ptr<pdcsu::util::DefArmSys> arm_sys_;
  std::shared_ptr<pdcsu::control::ICNORLearner> icnor_learner_;

  TurretPositionCalculator turret_pos_calc_;

  radps_t last_vel_ = 0.0_radps_;
  ms_t last_time_ = -1.0_ms_;
};
