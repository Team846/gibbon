#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <optional>
#include <variant>

#include "IntermediateController.h"
#include "pdcsu_units.h"

namespace funkit::control::hardware {

/*
TalonFX_interm

A class which interacts with the phoenix API to control and get information from
TalonFX hardware.
*/
class TalonFX_interm : public IntermediateController {
public:
  TalonFX_interm(int can_id, std::string_view bus = "",
      pdcsu::units::ms_t max_wait_time = pdcsu::units::ms_t{20},
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
  funkit::control::hardware::ControllerErrorCodes getErrorCode(
      ctre::phoenix::StatusCode code);

  base::ControlRequest last_command_;
  config::Gains gains_;

  std::optional<bool> last_brake_mode_{std::nullopt};
  std::optional<pdcsu::units::amp_t> last_motor_current_limit_{std::nullopt};
  std::optional<pdcsu::units::volt_t> last_voltage_compensation_{std::nullopt};
  std::optional<config::Gains> last_gains_{std::nullopt};
  std::optional<config::FollowerConfig> last_follower_config_{std::nullopt};

  ctre::phoenix6::hardware::TalonFX talon_;

  funkit::control::hardware::ControllerErrorCodes last_error_;

  units::millisecond_t
      max_wait_time_;  // Stored as WPILib for Phoenix API compatibility

  bool inverted_{false};
};

}  // namespace funkit::control::hardware