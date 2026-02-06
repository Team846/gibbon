#pragma once

#include <rev/AbsoluteEncoder.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>

#include <optional>
#include <variant>

#include "IntermediateController.h"
#include "funkit/control/hardware/Cooked.h"
#include "pdcsu_units.h"

namespace funkit::control::hardware {

/*
SparkMXFX_interm

A class which interacts with the REV API to control and get information from
SparkMAX or SparkFLEX hardware.
*/
class SparkMXFX_interm : public IntermediateController {
public:
  SparkMXFX_interm(int can_id, pdcsu::units::ms_t max_wait_time,
      bool is_controller_spark_flex, base::MotorMonkeyType mmtype,
      bool inverted = false);

  void Tick() override;

  void SetSoftLimits(pdcsu::units::radian_t forward_limit,
      pdcsu::units::radian_t reverse_limit) override;

  void SetGenome(config::MotorGenome genome, bool force_set = false) override;

  void EnableStatusFrames(config::StatusFrameSelections frames,
      pdcsu::units::ms_t faults_ms = pdcsu::units::ms_t{20},
      pdcsu::units::ms_t velocity_ms = pdcsu::units::ms_t{20},
      pdcsu::units::ms_t encoder_position_ms = pdcsu::units::ms_t{20},
      pdcsu::units::ms_t analog_position_ms = pdcsu::units::ms_t{20}) override;

  void OverrideStatusFramePeriod(
      config::StatusFrame frame, pdcsu::units::ms_t period) override;

  bool IsDuplicateControlMessage(base::ControlRequest cr) override;

  ReadResponse Read(ReadType type) override;

  void Write(base::ControlRequest cr) override;

  void SpecialConfigure(SpecialConfigureType type) override;

  ControllerErrorCodes GetLastErrorCode() override;

  bool VerifyConnected() override;

  void ZeroEncoder(pdcsu::units::radian_t position) override;

private:
  bool inverted_{false};

  funkit::control::hardware::ControllerErrorCodes getErrorCode(
      rev::REVLibError code);

  rev::spark::SparkBaseConfig configs{};

  base::ControlRequest last_command_;
  config::Gains gains_;

  std::optional<bool> last_brake_mode_{std::nullopt};
  std::optional<pdcsu::units::amp_t> last_motor_current_limit_{std::nullopt};
  std::optional<pdcsu::units::volt_t> last_voltage_compensation_{std::nullopt};
  std::optional<config::Gains> last_gains_{std::nullopt};
  std::optional<config::FollowerConfig> last_follower_config_{std::nullopt};

  rev::spark::SparkBase* esc_;
  rev::spark::SparkRelativeEncoder* encoder_;
  rev::spark::SparkClosedLoopController* pid_controller_;

  funkit::control::hardware::ControllerErrorCodes last_error_;

  int can_id_;

  Cooked cooked;
};

/*
SparkMAX_interm

A class which interacts with the REV API to control and get information from
SparkMAX hardware. Derives from SparkMXFX_interm.
*/
class SparkMAX_interm : public SparkMXFX_interm {
public:
  SparkMAX_interm(int can_id, pdcsu::units::ms_t max_wait_time,
      base::MotorMonkeyType mmtype, bool inverted);
};

/*
SparkFLEX_interm

A class which interacts with the REV API to control and get information from
SparkFLEX hardware. Derives from SparkMXFX_interm.
*/
class SparkFLEX_interm : public SparkMXFX_interm {
public:
  SparkFLEX_interm(int can_id, pdcsu::units::ms_t max_wait_time, bool inverted);
};

}  // namespace funkit::control::hardware