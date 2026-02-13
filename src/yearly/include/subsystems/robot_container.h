#pragma once

#include "funkit/robot/GenericRobotContainer.h"
#include "subsystems/abstract/control_input.h"
#include "subsystems/abstract/gpd.h"
#include "subsystems/hardware/DrivetrainConstructor.h"

#include "subsystems/hardware/ictest.h"
#include "subsystems/hardware/intake.h"
#include "subsystems/hardware/hoptake/hoptake_ss.h"
#include "subsystems/hardware/leds.h"
#include "subsystems/hardware/scorer/scorer_ss.h"

class RobotContainer : public funkit::robot::GenericRobotContainer {
public:
  LEDsSubsystem leds_{};

  DrivetrainConstructor drivetrain_constructor_{};
  funkit::robot::swerve::DrivetrainSubsystem drivetrain_{
      drivetrain_constructor_.getDrivetrainConfigs()};

  GPDSubsystem GPD_{&drivetrain_};

  ControlInputSubsystem control_input_{&drivetrain_};

  ScorerSuperstructure scorer_ss_{};

  HoptakeSuperstructure hoptake_ss_{};

  ICTestSubsystem ictest_{};

  RobotContainer() {
    RegisterPreference("init_drivetrain", true);
    RegisterPreference("init_leds", true);
    RegisterPreference("init_gpd", true);

    RegisterPreference("init_shooter", true);
    RegisterPreference("init_intake", true);
    RegisterPreference("init_ictest", true);

    bool drivetrain_init = (GetPreferenceValue_bool("init_drivetrain"));
    bool leds_init = (GetPreferenceValue_bool("init_leds"));
    bool gpd_init = (GetPreferenceValue_bool("init_gpd"));
    bool scorer_ss_init = (GetPreferenceValue_bool("init_scorer_ss"));
    bool hoptake_ss_init = (GetPreferenceValue_bool("init_hoptake_ss"));

    RegisterSubsystemGroupAB({{&control_input_, true}});
    RegisterSubsystemGroupA({{&leds_, leds_init}});

    RegisterSubsystemGroupAB({{&drivetrain_, drivetrain_init}});
    RegisterSubsystemGroupAB({{&GPD_, gpd_init}});

    // TODO: add intake_ss
    RegisterSubsystemGroupAB({{&scorer_ss_, scorer_ss_init}});
    RegisterSubsystemGroupAB({{&hoptake_ss_, hoptake_ss_init}});
  }
};
