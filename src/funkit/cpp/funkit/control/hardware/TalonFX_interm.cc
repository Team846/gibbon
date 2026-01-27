#include "funkit/control/hardware/TalonFX_interm.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "funkit/math/collection.h"
#include "pdcsu_units.h"

namespace funkit::control::hardware {

bool TalonFX_interm::VerifyConnected() { return talon_.IsAlive(); }

TalonFX_interm::TalonFX_interm(
    int can_id, std::string_view bus, pdcsu::units::ms_t max_wait_time)
    : talon_(can_id, ctre::phoenix6::CANBus{bus}),
      max_wait_time_(units::millisecond_t{max_wait_time.value()}) {}

void TalonFX_interm::Tick() {
  ctre::phoenix::StatusCode last_status_code = ctre::phoenix::StatusCode::OK;
  if (last_follower_config_.leader_CAN_id >= 0) {
    ctre::phoenix6::controls::Follower follower_msg{
        last_follower_config_.leader_CAN_id,
        (last_follower_config_.inverted)
            ? ctre::phoenix6::signals::MotorAlignmentValue::Opposed
            : ctre::phoenix6::signals::MotorAlignmentValue::Aligned};
    follower_msg.WithUpdateFreqHz(0_Hz);
    last_status_code = talon_.SetControl(follower_msg);
  } else if (double* dc = std::get_if<double>(&last_command_)) {
    ctre::phoenix6::controls::DutyCycleOut dc_msg{*dc};
    dc_msg.WithUpdateFreqHz(0_Hz);
    last_status_code = talon_.SetControl(dc_msg);
  } else if (pdcsu::units::radps_t* vel =
                 std::get_if<pdcsu::units::radps_t>(&last_command_)) {
    ctre::phoenix6::controls::VelocityVoltage vel_msg{
        units::radians_per_second_t{vel->value()}};
    vel_msg.WithUpdateFreqHz(0_Hz);
    last_status_code = talon_.SetControl(vel_msg);
  } else if (pdcsu::units::radian_t* pos =
                 std::get_if<pdcsu::units::radian_t>(&last_command_)) {
    ctre::phoenix6::controls::PositionVoltage pos_msg{
        units::radian_t{pos->value()}};
    pos_msg.WithUpdateFreqHz(0_Hz);
    last_status_code = talon_.SetControl(pos_msg);
  }
  last_error_ = getErrorCode(last_status_code);
}

void TalonFX_interm::SetSoftLimits(pdcsu::units::radian_t forward_limit,
    pdcsu::units::radian_t reverse_limit) {
  ctre::phoenix6::configs::SoftwareLimitSwitchConfigs configs{};
  configs.WithForwardSoftLimitEnable(true);
  configs.WithForwardSoftLimitThreshold(units::radian_t{forward_limit.value()});
  configs.WithReverseSoftLimitEnable(true);
  configs.WithReverseSoftLimitThreshold(units::radian_t{reverse_limit.value()});
  last_error_ = getErrorCode(talon_.GetConfigurator().Apply(configs));
}

void TalonFX_interm::SetGenome(config::MotorGenome genome) {
  if (last_brake_mode_ != genome.brake_mode) {
    talon_.SetNeutralMode(
        genome.brake_mode ? ctre::phoenix6::signals::NeutralModeValue::Brake
                          : ctre::phoenix6::signals::NeutralModeValue::Coast);
    last_brake_mode_ = genome.brake_mode;
  }

  if (!funkit::math::DEquals(last_motor_current_limit_.value(),
          genome.motor_current_limit.value())) {
    ctre::phoenix6::configs::CurrentLimitsConfigs current_configs{};
    current_configs.WithSupplyCurrentLimitEnable(false);
    current_configs.WithStatorCurrentLimitEnable(true);
    current_configs.WithStatorCurrentLimit(
        units::ampere_t{genome.motor_current_limit.value()});
    last_error_ = getErrorCode(talon_.GetConfigurator().Apply(current_configs));
    if (last_error_ != ControllerErrorCodes::kAllOK) return;
    last_motor_current_limit_ = genome.motor_current_limit;
  }

  if (!funkit::math::DEquals(last_voltage_compensation_.value(),
          genome.voltage_compensation.value())) {
    ctre::phoenix6::configs::VoltageConfigs voltage_configs{};
    units::volt_t voltage_wpi{genome.voltage_compensation.value()};
    voltage_configs.WithPeakForwardVoltage(voltage_wpi);
    voltage_configs.WithPeakReverseVoltage(-voltage_wpi);
    last_error_ = getErrorCode(talon_.GetConfigurator().Apply(voltage_configs));
    if (last_error_ != ControllerErrorCodes::kAllOK) return;
    last_voltage_compensation_ = genome.voltage_compensation;
  }

  if (!funkit::math::DEquals(last_gains_.kP, genome.gains.kP) ||
      !funkit::math::DEquals(last_gains_.kI, genome.gains.kI) ||
      !funkit::math::DEquals(last_gains_.kD, genome.gains.kD) ||
      !funkit::math::DEquals(last_gains_.kF, genome.gains.kF)) {
    gains_ = genome.gains;
    ctre::phoenix6::configs::Slot0Configs slot_configs{};
    slot_configs.WithKP(gains_.kP).WithKI(gains_.kI).WithKD(gains_.kD).WithKV(
        gains_.kF);
    last_error_ = getErrorCode(talon_.GetConfigurator().Apply(slot_configs));
    if (last_error_ != ControllerErrorCodes::kAllOK) return;
    last_gains_ = genome.gains;
  }

  if (last_follower_config_.leader_CAN_id !=
          genome.follower_config.leader_CAN_id ||
      last_follower_config_.inverted != genome.follower_config.inverted) {
    last_follower_config_ = genome.follower_config;
  }
}

void TalonFX_interm::Write(base::ControlRequest cr) { last_command_ = cr; }

void TalonFX_interm::EnableStatusFrames(config::StatusFrameSelections frames,
    pdcsu::units::ms_t faults_ms, pdcsu::units::ms_t velocity_ms,
    pdcsu::units::ms_t encoder_position_ms,
    pdcsu::units::ms_t analog_position_ms) {
  last_error_ =
      getErrorCode(talon_.OptimizeBusUtilization(0_Hz, max_wait_time_));
  if (last_error_ != ControllerErrorCodes::kAllOK) { return; }

  for (auto frame : frames) {
    ctre::phoenix::StatusCode last_status_code = ctre::phoenix::StatusCode::OK;
    switch (frame) {
    case funkit::control::config::StatusFrame::kCurrentFrame:
      last_status_code = talon_.GetSupplyCurrent().SetUpdateFrequency(
          units::hertz_t{1.0 / (velocity_ms.value() / 1000.0)}, max_wait_time_);
      break;
    case funkit::control::config::StatusFrame::kPositionFrame:
      last_status_code = talon_.GetPosition().SetUpdateFrequency(
          units::hertz_t{1.0 / (encoder_position_ms.value() / 1000.0)},
          max_wait_time_);
      if (last_status_code != ctre::phoenix::StatusCode::OK) break;

      [[fallthrough]];  // Latency compensation requires vel, acc frames
    case funkit::control::config::StatusFrame::kVelocityFrame:
      last_status_code = talon_.GetVelocity().SetUpdateFrequency(
          units::hertz_t{1.0 / (velocity_ms.value() / 1000.0)}, max_wait_time_);
      if (last_status_code != ctre::phoenix::StatusCode::OK) break;
      last_status_code = talon_.GetAcceleration().SetUpdateFrequency(
          units::hertz_t{1.0 / (velocity_ms.value() / 1000.0)}, max_wait_time_);
      break;

    default: break;
    }

    last_error_ = getErrorCode(last_status_code);
    if (last_error_ != ControllerErrorCodes::kAllOK) return;
  }
}

void TalonFX_interm::OverrideStatusFramePeriod(
    funkit::control::config::StatusFrame frame, pdcsu::units::ms_t period) {
  ctre::phoenix::StatusCode last_status_code = ctre::phoenix::StatusCode::OK;
  switch (frame) {
  case funkit::control::config::StatusFrame::kCurrentFrame:
    last_status_code = talon_.GetSupplyCurrent().SetUpdateFrequency(
        units::hertz_t{1.0 / (period.value() / 1000.0)}, max_wait_time_);
    break;
  case funkit::control::config::StatusFrame::kPositionFrame:
    last_status_code = talon_.GetPosition().SetUpdateFrequency(
        units::hertz_t{1.0 / (period.value() / 1000.0)}, max_wait_time_);
    if (last_status_code != ctre::phoenix::StatusCode::OK) break;

    [[fallthrough]];  // Latency compensation requires vel, acc frames
  case funkit::control::config::StatusFrame::kVelocityFrame:
    last_status_code = talon_.GetVelocity().SetUpdateFrequency(
        units::hertz_t{1.0 / (period.value() / 1000.0)}, max_wait_time_);
    if (last_status_code != ctre::phoenix::StatusCode::OK) break;
    last_status_code = talon_.GetAcceleration().SetUpdateFrequency(
        units::hertz_t{1.0 / (period.value() / 1000.0)}, max_wait_time_);
    break;
  default: break;
  }
  last_error_ = getErrorCode(last_status_code);
}

bool TalonFX_interm::IsDuplicateControlMessage(base::ControlRequest cr) {
  if (double* dc = std::get_if<double>(&last_command_)) {
    if (double* cr_dc = std::get_if<double>(&cr)) { return *dc == *cr_dc; }
    return false;
  } else if (pdcsu::units::radps_t* vel =
                 std::get_if<pdcsu::units::radps_t>(&last_command_)) {
    if (pdcsu::units::radps_t* cr_vel =
            std::get_if<pdcsu::units::radps_t>(&cr)) {
      return vel->value() == cr_vel->value();
    }
    return false;
  } else if (pdcsu::units::radian_t* pos =
                 std::get_if<pdcsu::units::radian_t>(&last_command_)) {
    if (pdcsu::units::radian_t* cr_pos =
            std::get_if<pdcsu::units::radian_t>(&cr)) {
      return pos->value() == cr_pos->value();
    }
    return false;
  }
  return false;
}

ReadResponse TalonFX_interm::Read(ReadType type) {
  switch (type) {
  case ReadType::kReadPosition: {
    auto pos_wpi = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
        talon_.GetPosition(), talon_.GetVelocity());
    return pos_wpi.to<double>() * 2.0 * M_PI;
  }
  case ReadType::kReadVelocity: {
    auto vel_wpi = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
        talon_.GetVelocity(), talon_.GetAcceleration());
    return vel_wpi.to<double>() * 2.0 * M_PI;
  }
  case ReadType::kReadCurrent:
    return talon_.GetSupplyCurrent().GetValue().to<double>();
  case ReadType::kFwdSwitch:
    return (talon_.GetForwardLimit(true).GetValue() ==
               ctre::phoenix6::signals::ForwardLimitValue::ClosedToGround)
               ? 1.0
               : -1.0;
  case ReadType::kRevSwitch:
    return (talon_.GetReverseLimit(true).GetValue() ==
               ctre::phoenix6::signals::ReverseLimitValue::ClosedToGround)
               ? 1.0
               : -1.0;
  case ReadType::kTemperature:
    return talon_.GetDeviceTemp().GetValue().to<double>();
  case ReadType::kAbsPosition:
    throw std::runtime_error(
        "ReadType absolute error not implemented for TalonFX");
  default: return 0.0;
  }
}

void TalonFX_interm::SpecialConfigure(SpecialConfigureType type) {
  ctre::phoenix6::configs::HardwareLimitSwitchConfigs configs{};
  ctre::phoenix6::signals::ForwardLimitTypeValue fwd_type;
  ctre::phoenix6::signals::ReverseLimitTypeValue rev_type;

  if (type == base::LimitSwitchDefaultState::kNormallyOff) {
    fwd_type = ctre::phoenix6::signals::ForwardLimitTypeValue::NormallyClosed;
    rev_type = ctre::phoenix6::signals::ReverseLimitTypeValue::NormallyClosed;
  } else {
    fwd_type = ctre::phoenix6::signals::ForwardLimitTypeValue::NormallyOpen;
    rev_type = ctre::phoenix6::signals::ReverseLimitTypeValue::NormallyOpen;
  }

  configs.WithForwardLimitEnable(true)
      .WithForwardLimitType(fwd_type)
      .WithReverseLimitEnable(true)
      .WithReverseLimitType(rev_type);
  last_error_ = getErrorCode(talon_.GetConfigurator().Apply(configs));
}

ControllerErrorCodes TalonFX_interm::GetLastErrorCode() {
  ControllerErrorCodes toReturn = last_error_;
  last_error_ = ControllerErrorCodes::kAllOK;
  return toReturn;
}

void TalonFX_interm::ZeroEncoder(pdcsu::units::radian_t position) {
  last_error_ =
      getErrorCode(talon_.SetPosition(units::radian_t{position.value()}));
}

funkit::control::hardware::ControllerErrorCodes TalonFX_interm::getErrorCode(
    ctre::phoenix::StatusCode code) {
  switch (code) {
  case ctre::phoenix::StatusCode::OK: return ControllerErrorCodes::kAllOK;
  case ctre::phoenix::StatusCode::ApiTooOld:
  case ctre::phoenix::StatusCode::FirmwareTooOld:
  case ctre::phoenix::StatusCode::FirmwareTooNew:
  case ctre::phoenix::StatusCode::FirmwareVersNotCompatible:
    return ControllerErrorCodes::kConfigFailed;
  case ctre::phoenix::StatusCode::TxFailed:
  case ctre::phoenix::StatusCode::CouldNotSendCanFrame:
  case ctre::phoenix::StatusCode::CanOverflowed:
    return ControllerErrorCodes::kCANDisconnected;
  case ctre::phoenix::StatusCode::CanMessageStale:
  case ctre::phoenix::StatusCode::RxTimeout:
  case ctre::phoenix::StatusCode::TxTimeout:
    return ControllerErrorCodes::kCANMessageStale;
  case ctre::phoenix::StatusCode::InvalidDeviceSpec:
  case ctre::phoenix::StatusCode::EcuIsNotPresent:
  case ctre::phoenix::StatusCode::NodeIsInvalid:
  case ctre::phoenix::StatusCode::DeviceIsNull:
  case ctre::phoenix::StatusCode::NoDevicesOnBus:
    return ControllerErrorCodes::kDeviceDisconnected;
  case ctre::phoenix::StatusCode::ConfigFailed:
  case ctre::phoenix::StatusCode::ConfigReadWriteMismatch:
  case ctre::phoenix::StatusCode::CouldNotReqSetConfigs:
  case ctre::phoenix::StatusCode::InvalidParamValue:
    return ControllerErrorCodes::kConfigFailed;
  case ctre::phoenix::StatusCode::InvalidIDToFollow:
  case ctre::phoenix::StatusCode::CouldNotFindDynamicId:
    return ControllerErrorCodes::kFollowingError;
  case ctre::phoenix::StatusCode::TimeoutIso15Response:
  case ctre::phoenix::StatusCode::TimeoutCannotBeZero:
    return ControllerErrorCodes::kTimeout;
  }

  if (code.IsWarning()) {
    return ControllerErrorCodes::kWarning;
  } else if (code.IsError()) {
    return ControllerErrorCodes::kError;
  }
  return ControllerErrorCodes::kAllOK;
}

}  // namespace funkit::control::hardware