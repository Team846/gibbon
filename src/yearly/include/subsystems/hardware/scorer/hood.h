#pragma once

#include <deque>
#include <memory>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericRobot.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/wpilib/time.h"
#include "pdcsu_control.h"

struct HoodReadings {
  degree_t pos_;
  degps_t vel_;
  bool in_position_;
};

struct HoodTarget {
  degree_t pos_;
  degps_t vel_;
};

class HoodSubsystem
    : public funkit::robot::GenericSubsystem<HoodReadings, HoodTarget> {
public:
  HoodSubsystem();
  ~HoodSubsystem();

  void Setup() override;

  HoodTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void ZeroEncoders();
  void ZeroWithAbsoluteEncoder(bool retry = true);

private:
  HoodReadings ReadFromHardware() override;
  void WriteToHardware(HoodTarget target) override;

  funkit::control::HigherMotorController esc_;

  std::unique_ptr<pdcsu::control::ICNORPositionControl> icnor_controller_;
  std::unique_ptr<pdcsu::util::DefArmSys> arm_sys_;
  std::shared_ptr<pdcsu::control::ICNORLearner> icnor_learner_;
};
