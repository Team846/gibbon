#include "funkit/control/hardware/SparkMXFX_interm.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "funkit/control/hardware/Cooked.h"
#include "funkit/math/collection.h"
#include "pdcsu_units.h"

#define vector_has(vec, val) std::find(vec.begin(), vec.end(), val) != vec.end()

namespace funkit::control::hardware {

#define set_last_error(code) last_error_ = getErrorCode(code)

#define CONFIG_RESET rev::ResetMode::kResetSafeParameters
#define NO_CONFIG_RESET rev::ResetMode::kNoResetSafeParameters

#define PERSIST_PARAMS rev::PersistMode::kPersistParameters
#define NO_PERSIST_PARAMS rev::PersistMode::kNoPersistParameters

#define APPLY_CONFIG_NO_RESET() \
  set_last_error(esc_->Configure(configs, NO_CONFIG_RESET, NO_PERSIST_PARAMS))

bool SparkMXFX_interm::VerifyConnected() {
  if (esc_ == nullptr) return false;
  esc_->GetFirmwareVersion();
  return esc_->GetFirmwareVersion() != 0;
}

SparkMXFX_interm::SparkMXFX_interm(int can_id, pdcsu::units::ms_t max_wait_time,
    bool is_controller_spark_flex, base::MotorMonkeyType mmtype, bool inverted)
    : inverted_{inverted}, can_id_{can_id}, cooked{mmtype} {
  esc_ = is_controller_spark_flex
             ? static_cast<rev::spark::SparkBase*>(new rev::spark::SparkFlex{
                   can_id, rev::spark::SparkFlex::MotorType::kBrushless})
             : new rev::spark::SparkMax{
                   can_id, rev::spark::SparkMax::MotorType::kBrushless};
  configs.Inverted(inverted);
  encoder_ = new rev::spark::SparkRelativeEncoder{esc_->GetEncoder()};
  pid_controller_ = new rev::spark::SparkClosedLoopController{
      esc_->GetClosedLoopController()};

  set_last_error(esc_->Configure(configs, CONFIG_RESET, NO_PERSIST_PARAMS));
  set_last_error(esc_->SetCANTimeout(static_cast<int>(max_wait_time.value())));
}

void SparkMXFX_interm::Tick() {
  rev::REVLibError last_status_code = rev::REVLibError::kOk;
  if (double* dc = std::get_if<double>(&last_command_)) {
    double dc_u = *dc;
    dc_u = cooked.Record(dc_u, radps_t(Read(ReadType::kReadVelocity)),
        Read(ReadType::kTemperature));
    last_status_code = pid_controller_->SetSetpoint(
        dc_u, rev::spark::SparkBase::ControlType::kDutyCycle);
    esc_->Set(dc_u);
  } else if (pdcsu::units::radps_t* vel =
                 std::get_if<pdcsu::units::radps_t>(&last_command_)) {
    units::revolutions_per_minute_t rev_ms_t{
        vel->value() * 60.0 / (2.0 * 3.14159265358979323846)};
    last_status_code = pid_controller_->SetSetpoint(
        rev_ms_t.to<double>(), rev::spark::SparkBase::ControlType::kVelocity);
  } else if (pdcsu::units::radian_t* pos =
                 std::get_if<pdcsu::units::radian_t>(&last_command_)) {
    units::turn_t pos_ms_t{pos->value() / (2.0 * 3.14159265358979323846)};
    last_status_code = pid_controller_->SetSetpoint(
        pos_ms_t.to<double>(), rev::spark::SparkBase::ControlType::kPosition);
  }
  set_last_error(last_status_code);
}

void SparkMXFX_interm::SetSoftLimits(pdcsu::units::radian_t forward_limit,
    pdcsu::units::radian_t reverse_limit) {
  units::turn_t limit_fwd{
      forward_limit.value() / (2.0 * 3.14159265358979323846)};
  units::turn_t limit_rev{
      reverse_limit.value() / (2.0 * 3.14159265358979323846)};
  configs.softLimit.ForwardSoftLimitEnabled(true)
      .ForwardSoftLimit(limit_fwd.to<double>())
      .ReverseSoftLimitEnabled(true)
      .ReverseSoftLimit(limit_rev.to<double>());

  APPLY_CONFIG_NO_RESET();
}

void SparkMXFX_interm::SetGenome(config::MotorGenome genome) {
  bool config_changed = false;

  if (last_brake_mode_ != genome.brake_mode) {
    configs.SetIdleMode(genome.brake_mode
                            ? rev::spark::SparkBaseConfig::IdleMode::kBrake
                            : rev::spark::SparkBaseConfig::IdleMode::kCoast);
    last_brake_mode_ = genome.brake_mode;
    config_changed = true;
  }

  if (!funkit::math::DEquals(last_motor_current_limit_.value(),
          genome.motor_current_limit.value())) {
    configs.SmartCurrentLimit(
        static_cast<int>(genome.motor_current_limit.value()));
    last_motor_current_limit_ = genome.motor_current_limit;
    config_changed = true;
  }

  if (!funkit::math::DEquals(last_voltage_compensation_.value(),
          genome.voltage_compensation.value())) {
    configs.VoltageCompensation(genome.voltage_compensation.value());
    last_voltage_compensation_ = genome.voltage_compensation;
    config_changed = true;
  }

  if (!funkit::math::DEquals(last_gains_.kP, genome.gains.kP) ||
      !funkit::math::DEquals(last_gains_.kI, genome.gains.kI) ||
      !funkit::math::DEquals(last_gains_.kD, genome.gains.kD) ||
      !funkit::math::DEquals(last_gains_.kF, genome.gains.kF)) {
    gains_ = genome.gains;
    configs.closedLoop.Pid(gains_.kP, gains_.kI, std::abs(gains_.kD));
    configs.closedLoop.feedForward.kV(std::abs(gains_.kF));
    last_gains_ = genome.gains;
    config_changed = true;
  }

  if (config_changed) { APPLY_CONFIG_NO_RESET(); }
}

void SparkMXFX_interm::Write(base::ControlRequest cr) { last_command_ = cr; }

void SparkMXFX_interm::EnableStatusFrames(config::StatusFrameSelections frames,
    pdcsu::units::ms_t faults_ms, pdcsu::units::ms_t velocity_ms,
    pdcsu::units::ms_t encoder_position_ms,
    pdcsu::units::ms_t analog_position_ms) {
  if (vector_has(frames, config::StatusFrame::kLeader) ||
      vector_has(frames, config::StatusFrame::kFaultFrame)) {
    configs.signals.FaultsPeriodMs(static_cast<int>(faults_ms.value()));
    configs.signals.FaultsAlwaysOn(true);
  } else {
    configs.signals.FaultsPeriodMs(32767);
    configs.signals.FaultsAlwaysOn(false);
  }

  if (vector_has(frames, config::StatusFrame::kVelocityFrame) ||
      vector_has(frames, config::StatusFrame::kCurrentFrame) ||
      true) {  // Forced to true for current monitoring
    configs.signals.PrimaryEncoderVelocityPeriodMs(
        static_cast<int>(velocity_ms.value()));
    configs.signals.PrimaryEncoderVelocityAlwaysOn(true);
    configs.signals.OutputCurrentPeriodMs(
        static_cast<int>(velocity_ms.value()));
    // configs.signals.OutputCurrentAlwaysOn(true);
    // configs.signals.MotorTemperatureAlwaysOn(true);
    configs.signals.MotorTemperaturePeriodMs(
        1000);  // Temperature required less often
  } else {
    configs.signals.PrimaryEncoderVelocityPeriodMs(32767);
    configs.signals.PrimaryEncoderVelocityAlwaysOn(false);
  }

  if (vector_has(frames, config::StatusFrame::kPositionFrame)) {
    configs.signals.PrimaryEncoderPositionPeriodMs(
        static_cast<int>(encoder_position_ms.value()));
    configs.signals.PrimaryEncoderPositionAlwaysOn(true);
  } else {
    configs.signals.PrimaryEncoderPositionPeriodMs(32767);
    configs.signals.PrimaryEncoderPositionAlwaysOn(false);
  }

  if (vector_has(frames, config::StatusFrame::kSensorFrame)) {
    configs.signals.AnalogPositionPeriodMs(
        static_cast<int>(analog_position_ms.value()));
    configs.signals.AnalogPositionAlwaysOn(true);
    configs.signals.LimitsPeriodMs(static_cast<int>(faults_ms.value()));
    // configs.signals.LimitsAlwaysOn(true);
  } else {
    configs.signals.AnalogPositionPeriodMs(32767);
    configs.signals.AnalogPositionAlwaysOn(false);
    configs.signals.LimitsPeriodMs(32767);
    // configs.signals.LimitsAlwaysOn(false);
  }

  if (vector_has(frames, config::StatusFrame::kAbsoluteFrame)) {
    configs.absoluteEncoder.SetSparkMaxDataPortConfig();
  }

  APPLY_CONFIG_NO_RESET();
}

void SparkMXFX_interm::OverrideStatusFramePeriod(
    funkit::control::config::StatusFrame frame, pdcsu::units::ms_t period) {
  if (frame == config::StatusFrame::kFaultFrame ||
      frame == config::StatusFrame::kLeader) {
    configs.signals.FaultsPeriodMs(static_cast<int>(period.value()));
    configs.signals.FaultsAlwaysOn(true);
  } else if (frame == config::StatusFrame::kVelocityFrame) {
    configs.signals.PrimaryEncoderVelocityPeriodMs(
        static_cast<int>(period.value()));
  } else if (frame == config::StatusFrame::kPositionFrame) {
    configs.signals.PrimaryEncoderPositionPeriodMs(
        static_cast<int>(period.value()));
  } else if (frame == config::StatusFrame::kSensorFrame) {
    configs.signals.AnalogPositionPeriodMs(20);
  }
  APPLY_CONFIG_NO_RESET();
}

bool SparkMXFX_interm::IsDuplicateControlMessage(base::ControlRequest cr) {
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

ReadResponse SparkMXFX_interm::Read(ReadType type) {
  switch (type) {
  case ReadType::kReadPosition: {
    auto pos_turn = units::make_unit<units::turn_t>(encoder_->GetPosition());
    return (pos_turn.to<double>() * 2.0 * 3.14159265358979323846);
  }
  case ReadType::kReadVelocity: {
    auto vel_rpm = UnitDivision<rotation_t, minute_t>(encoder_->GetVelocity());
    return (radps_t(vel_rpm).value());
  }
  case ReadType::kReadCurrent: return esc_->GetOutputCurrent();
  case ReadType::kFwdSwitch:
    return esc_->GetForwardLimitSwitch().Get() ? 1.0 : -1.0;
  case ReadType::kRevSwitch:
    return esc_->GetReverseLimitSwitch().Get() ? 1.0 : -1.0;
  case ReadType::kAbsPosition: {
    auto turn_wpi = units::make_unit<units::turn_t>(
        esc_->GetAbsoluteEncoder().GetPosition());
    return turn_wpi.to<double>();
  }
  case ReadType::kTemperature: return esc_->GetMotorTemperature();
  default: return 0.0;
  }
}

void SparkMXFX_interm::SpecialConfigure(SpecialConfigureType type) {
  rev::spark::LimitSwitchConfig::Type d_type;
  if (type == base::LimitSwitchDefaultState::kNormallyOn) {
    d_type = rev::spark::LimitSwitchConfig::Type::kNormallyClosed;
  } else {
    d_type = rev::spark::LimitSwitchConfig::Type::kNormallyOpen;
  }

  configs.limitSwitch
      .ForwardLimitSwitchTriggerBehavior(
          rev::spark::LimitSwitchConfig::Behavior::kKeepMovingMotor)
      .ForwardLimitSwitchType(d_type)
      .ReverseLimitSwitchTriggerBehavior(
          rev::spark::LimitSwitchConfig::Behavior::kKeepMovingMotor)
      .ReverseLimitSwitchType(d_type);

  APPLY_CONFIG_NO_RESET();
}

ControllerErrorCodes SparkMXFX_interm::GetLastErrorCode() {
  ControllerErrorCodes toReturn = last_error_;
  last_error_ = ControllerErrorCodes::kAllOK;
  return toReturn;
}

funkit::control::hardware::ControllerErrorCodes SparkMXFX_interm::getErrorCode(
    rev::REVLibError code) {
  switch (code) {
  case rev::REVLibError::kOk: return ControllerErrorCodes::kAllOK;
  case rev::REVLibError::kInvalidCANId:
  case rev::REVLibError::kDuplicateCANId:
    return ControllerErrorCodes::kInvalidCANID;
  case rev::REVLibError::kTimeout: return ControllerErrorCodes::kTimeout;
  case rev::REVLibError::kHALError: return ControllerErrorCodes::kHALError;
  case rev::REVLibError::kFollowConfigMismatch:
    return ControllerErrorCodes::kFollowingError;
  case rev::REVLibError::kCANDisconnected:
    return ControllerErrorCodes::kCANDisconnected;
  case rev::REVLibError::kCantFindFirmware:
  case rev::REVLibError::kFirmwareTooOld:
  case rev::REVLibError::kFirmwareTooNew:
    return ControllerErrorCodes::kVersionMismatch;
  case rev::REVLibError::kParamInvalidID:
  case rev::REVLibError::kParamMismatchType:
  case rev::REVLibError::kParamAccessMode:
  case rev::REVLibError::kParamInvalid:
  case rev::REVLibError::kSparkMaxDataPortAlreadyConfiguredDifferently:
    return ControllerErrorCodes::kConfigFailed;
  case rev::REVLibError::kError: return ControllerErrorCodes::kError;
  case rev::REVLibError::kParamNotImplementedDeprecated:
  case rev::REVLibError::kNotImplemented:
  case rev::REVLibError::kUnknown:
  default: return ControllerErrorCodes::kWarning;
  }
}

void SparkMXFX_interm::ZeroEncoder(pdcsu::units::radian_t position) {
  set_last_error(
      encoder_->SetPosition(position.value() / (2.0 * 3.14159265358979323846)));
}

SparkMAX_interm::SparkMAX_interm(int can_id, pdcsu::units::ms_t max_wait_time,
    base::MotorMonkeyType mmtype, bool inverted)
    : SparkMXFX_interm(can_id, max_wait_time, false, mmtype, inverted) {}

SparkFLEX_interm::SparkFLEX_interm(
    int can_id, pdcsu::units::ms_t max_wait_time, bool inverted)
    : SparkMXFX_interm(can_id, max_wait_time, true,
          base::MotorMonkeyType::SPARK_FLEX_VORTEX, inverted) {}

}  // namespace funkit::control::hardware