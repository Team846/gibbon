#pragma once

#include "funkit/robot/GenericRobotContainer.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/abstract/gpd.h"
#include "subsystems/hardware/DrivetrainConstructor.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/leds.h"
#include "subsystems/hardware/shooter.h"
#include "subsystems/hardware/testcrt.h"

class RobotContainer : public funkit::robot::GenericRobotContainer {
public:
  LEDsSubsystem leds_{};

  DrivetrainConstructor drivetrain_constructor_{};
  funkit::robot::swerve::DrivetrainSubsystem drivetrain_{
      drivetrain_constructor_.getDrivetrainConfigs()};

  GPDSubsystem GPD_{&drivetrain_};

  ControlInputSubsystem control_input_{&drivetrain_};

  TurretTestSubsystem turr_test{};

  ShooterSubsystem shooter_{};

  IntakeSubsystem intake_{};

  RobotContainer() {
    RegisterPreference("init_drivetrain", true);
    RegisterPreference("init_leds", true);
    RegisterPreference("init_gpd", true);
    RegisterPreference("init_shooter", true);
    RegisterPreference("init_intake", true);

    bool drivetrain_init = (GetPreferenceValue_bool("init_drivetrain"));
    bool leds_init = (GetPreferenceValue_bool("init_leds"));
    bool gpd_init = (GetPreferenceValue_bool("init_gpd"));

    RegisterSubsystemGroupAB({{&control_input_, true}});
    RegisterSubsystemGroupA({{&leds_, leds_init}});

    RegisterSubsystemGroupAB({{&drivetrain_, drivetrain_init}});
    RegisterSubsystemGroupAB({{&GPD_, gpd_init}});

    // bool shooter_init = (GetPreferenceValue_bool("init_shooter"));
    // bool intake_init = (GetPreferenceValue_bool("init_intake"));

    // RegisterSubsystemGroupAB({{&shooter_, shooter_init}});
    // RegisterSubsystemGroupAB({{&intake_, intake_init}});

    // RegisterSubsystemGroupAB({{&turr_test, true}});
  }
};
