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

  bool diagonalize_bump;
  double intake;
  bool climb_align;
  bool point_blank_shot;

  // Operator Inputs
  bool override_autoshoot;
  bool force_shoot;
  bool agitate;
  bool hood_trim_cw;
  bool hood_trim_ccw;
  bool turret_trim_cw;
  bool turret_trim_ccw;
  bool pass_mode;
  bool descend_l1;
  bool override_force_assist;
  bool evac_storage;
  bool rev_dye_rotor;
  bool home;

  // Operator Keyboard Overrides
  bool turret_no_spin;
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
