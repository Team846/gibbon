#pragma once

#include "funkit/control/config/genome.h"

struct ports {
  struct driver_ {
    static constexpr int kXbox_DSPort = 0;
  };

  struct operator_ {
    static constexpr int kXbox_DSPort = 1;
  };

  struct drivetrain_ {
    static constexpr int kFRDrive_CANID = 2;
    static constexpr int kFLDrive_CANID = 5;
    static constexpr int kBLDrive_CANID = 8;
    static constexpr int kBRDrive_CANID = 11;

    static constexpr int kFLSteer_CANID = 7;
    static constexpr int kFRSteer_CANID = 4;
    static constexpr int kBLSteer_CANID = 10;
    static constexpr int kBRSteer_CANID = 13;

    static constexpr int kFRCANCoder_CANID = 3;
    static constexpr int kFLCANCoder_CANID = 6;
    static constexpr int kBLCANCoder_CANID = 9;
    static constexpr int kBRCANCoder_CANID = 12;

    static constexpr int kPIGEON_CANID = 14;
  };

  struct leds_ {
    static constexpr int kLEDStrip1 = 6;
  };

  struct turret_ {
    static constexpr funkit::control::config::MotorConstructionParameters
        kTurretParams = {15, "", false};  // TODO: add to canivore
  };

  struct shooter_ {
    static constexpr funkit::control::config::MotorConstructionParameters
        kShooter1Params = {16, "", false};
    static constexpr funkit::control::config::MotorConstructionParameters
        kShooter2Params = {17, "", true};
  };

  struct hood_ {
    static constexpr funkit::control::config::MotorConstructionParameters
        kHoodParams = {18, "", false};
  };

  struct intake_ {
    static constexpr funkit::control::config::MotorConstructionParameters
        kIntakeParams = {19, "", false};
  };

  struct dye_rotor_ {
    static constexpr funkit::control::config::MotorConstructionParameters
        kDyeRotorParams = {20, "", false};
  };

  struct pivot_ {
    static constexpr funkit::control::config::MotorConstructionParameters
        kPivotParams = {21, "", false};
  };
  struct ictest_ {
    static constexpr int kMotor1_CANID = 51;
    static constexpr int kCANCoder1_CANID = 52;
    static constexpr int kCANCoder2_CANID = 53;
  };
};
