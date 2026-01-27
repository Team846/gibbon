#pragma once

#include "funkit/robot/GenericController.h"
#include "funkit/robot/GenericSubsystem.h"
#include "funkit/robot/swerve/drivetrain.h"
#include "funkit/robot/xbox.h"

struct ControlInputReadings {
  // Driving
  double translate_x;
  double translate_y;

  double rotation;

  bool zero_bearing;

  // Shooting controls
  bool prepare_shot;
  bool shoot;
};

struct ControlInputTarget {
  bool driver_rumble;
  bool operator_rumble;
};

class ControlInputSubsystem
    : public funkit::robot::GenericSubsystem<ControlInputReadings,
          ControlInputTarget> {
public:
  ControlInputSubsystem(funkit::robot::swerve::DrivetrainSubsystem* drivetrain);

  void Setup() override;

  ControlInputTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  ControlInputReadings UpdateWithInput();

private:
  ControlInputReadings previous_readings_{};

  funkit::robot::swerve::DrivetrainSubsystem* drivetrain_ss_;

  funkit::robot::XboxReadings previous_driver_{};
  funkit::robot::XboxReadings previous_operator_{};
  funkit::robot::GenericControllerReadings previous_operator_keyboard_{};

  frc::XboxController driver_{0};
  frc::XboxController operator_{1};
  frc::GenericHID operator_keyboard_{2};

  ControlInputReadings ReadFromHardware() override;

  void WriteToHardware(ControlInputTarget target) override;
};
