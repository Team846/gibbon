#pragma once

#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

using inches_per_second_t = units::unit_t<
    units::compound_unit<units::inches, units::inverse<units::seconds>>>;

struct ICTestReadings {
  units::inch_t pos;
  inches_per_second_t vel;
};

struct ICTestTarget {
  units::inch_t pos;
};

class ICTestSubsystem
    : public funkit::robot::GenericSubsystem<ICTestReadings, ICTestTarget> {
public:
  ICTestSubsystem();
  ~ICTestSubsystem();

  void Setup() override;

  ICTestTarget ZeroTarget() const override;

  bool VerifyHardware() override;

private:
  ICTestReadings ReadFromHardware() override;

  void WriteToHardware(ICTestTarget target) override;

  funkit::control::HigherMotorController esc_1_;
  funkit::control::HigherMotorController esc_2_;
  std::unique_ptr<pdcsu::control::ICNORPositionControl> icnor_controller_;
  std::unique_ptr<pdcsu::util::DefLinearSys> linear_sys_;
  std::shared_ptr<pdcsu::control::ICNORLearner> icnor_learner_;

  double max_vel_mps_;
};
