#pragma once

#include <optional>
#include <stdexcept>
#include <type_traits>
#include <variant>
#include <vector>

#include "funkit/control/MonkeyMaster.h"
#include "funkit/control/base/motor_control_base.h"
#include "funkit/control/base/motor_specs.h"
#include "funkit/control/config/soft_limits.h"
#include "pdcsu_control.h"
#include "pdcsu_units.h"

namespace funkit::control {

/*
HigherMotorController

A class that interfaces with MonkeyMaster to provide higher-level control of
motors.
*/
class HigherMotorController {
public:
  HigherMotorController(
      base::MotorMonkeyType mmtype, config::MotorConstructionParameters params);

  // Sets up the motor. Gets a slot ID from MonkeyMaster.
  void Setup(config::MotorGenome genome,
      std::variant<pdcsu::util::DefLinearSys, pdcsu::util::DefArmSys> plant);

  void ModifyGenome(config::MotorGenome genome);

  /*
  WriteDC()

  Writes a duty cycle directly to the motor controller. Value should be in the
  range [-1.0, 1.0].
  */
  void WriteDC(double duty_cycle);

  /*
  WriteVelocity()

  Writes a velocity setpoint to the motor controller. Control loop executed
  locally.
  */
  void WriteVelocity(mps_t velocity);
  void WriteVelocity(radps_t velocity);

  /*
  WritePosition()

  Writes a position setpoint to the motor controller. Control loop executed
  locally.
  */
  void WritePosition(meter_t position);
  void WritePosition(radian_t position);

  /*
  WriteVelocityOnController()

  Writes a velocity setpoint to the motor controller. Control loop executed
  onboard the motor controller.
  */
  void WriteVelocityOnController(mps_t velocity);
  void WriteVelocityOnController(radps_t velocity);

  /*
  WritePositionOnController()

  Writes a position setpoint to the motor controller. Control loop executed
  onboard the motor controller.
  */
  void WritePositionOnController(meter_t position);
  void WritePositionOnController(radian_t position);

  template <typename VelUnit> VelUnit GetVelocity();

  template <typename PosUnit> PosUnit GetPosition();

  amp_t GetCurrent();

  /*
  SetPosition()

  Zeroes the encoder to the specified value. Converts from real units to native
  units using the plant.
  */
  void SetPosition(meter_t position);
  void SetPosition(radian_t position);

  void SetLoad(nm_t load);

  bool VerifyConnected();

  // Soft limits maintained by the motor controller
  void SetControllerSoftLimits(radian_t forward_limit, radian_t reverse_limit);

  // Custom soft limits maintained by HigherMotorController
  void SetSoftLimits(config::SoftLimits soft_limits);

  void EnableStatusFrames(std::vector<config::StatusFrame> frames,
      ms_t faults_ms = 20_u_ms, ms_t velocity_ms = 20_u_ms,
      ms_t encoder_position_ms = 20_u_ms, ms_t analog_position_ms = 20_u_ms);

  void OverrideStatusFramePeriod(config::StatusFrame frame, ms_t period);

  void SpecialConfiguration(control::hardware::SpecialConfigureType type);

  /* Only limit switches and absolute encoder positions have been implemented */
  hardware::ReadResponse SpecialRead(hardware::ReadType type);

private:
  funkit::control::base::MotorMonkeyType mmtype_;
  config::MotorConstructionParameters constr_params_;
  std::optional<std::variant<pdcsu::util::DefLinearSys, pdcsu::util::DefArmSys>>
      plant_;

  config::MotorGenome genome_;

  hardware::IntermediateController* intermediate_controller_;

  size_t slot_id_;

  std::optional<config::SoftLimits> soft_limits_;
};

inline void HigherMotorController::WriteVelocity(mps_t velocity) {
  if (!plant_.has_value()) {
    throw std::runtime_error("Plant must be set before using WriteVelocity");
  }

  radps_t velocity_native;
  if (auto* linear = std::get_if<pdcsu::util::DefLinearSys>(&plant_.value())) {
    velocity_native = linear->toNative(velocity);
  } else {
    throw std::runtime_error(
        "WriteVelocity<mps_t> requires DefLinearSys plant");
  }

  if (soft_limits_.has_value()) {
    auto response_pos =
        MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadPosition);
    radian_t current_pos_native{response_pos};
    velocity_native =
        soft_limits_.value().LimitVelocity(velocity_native, current_pos_native);
  }

  auto response_vel =
      MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadVelocity);
  radps_t current_velocity_native{response_vel};
  auto error = velocity_native.value() - current_velocity_native.value();

  double dc_target = genome_.gains.kP * error + genome_.gains.kI * 0.0 +
                     genome_.gains.kD * 0.0 +
                     genome_.gains.kF * velocity_native.value();

  MonkeyMaster::WriteDC(slot_id_, dc_target);
}

inline void HigherMotorController::WriteVelocity(radps_t velocity) {
  if (!plant_.has_value()) {
    throw std::runtime_error("Plant must be set before using WriteVelocity");
  }

  radps_t velocity_native;
  if (auto* arm = std::get_if<pdcsu::util::DefArmSys>(&plant_.value())) {
    velocity_native = arm->toNative(velocity);
  } else {
    throw std::runtime_error("WriteVelocity<radps_t> requires DefArmSys plant");
  }

  if (soft_limits_.has_value()) {
    auto response_pos =
        MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadPosition);
    radian_t current_pos_native{response_pos};
    velocity_native =
        soft_limits_.value().LimitVelocity(velocity_native, current_pos_native);
  }

  auto response_vel =
      MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadVelocity);
  radps_t current_velocity_native{response_vel};
  auto error = velocity_native.value() - current_velocity_native.value();

  double dc_target = genome_.gains.kP * error + genome_.gains.kI * 0.0 +
                     genome_.gains.kD * 0.0 +
                     genome_.gains.kF * velocity_native.value();

  MonkeyMaster::WriteDC(slot_id_, dc_target);
}

inline void HigherMotorController::WritePosition(meter_t position) {
  if (!plant_.has_value()) {
    throw std::runtime_error("Plant must be set before using WritePosition");
  }

  radian_t position_native;
  if (auto* linear = std::get_if<pdcsu::util::DefLinearSys>(&plant_.value())) {
    position_native = linear->toNative(position);
  } else {
    throw std::runtime_error(
        "WritePosition<meter_t> requires DefLinearSys plant");
  }

  if (soft_limits_.has_value()) {
    position_native = soft_limits_.value().LimitPosition(position_native);
  }

  auto response_pos =
      MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadPosition);
  radian_t current_position_native{response_pos};
  auto response_vel =
      MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadVelocity);
  radps_t current_velocity_native{response_vel};
  auto error = position_native.value() - current_position_native.value();

  double dc_target = genome_.gains.kP * error + genome_.gains.kI * 0.0 +
                     genome_.gains.kD * current_velocity_native.value() +
                     genome_.gains.kF * 0.0;

  MonkeyMaster::WriteDC(slot_id_, dc_target);
}

inline void HigherMotorController::WritePosition(radian_t position) {
  if (!plant_.has_value()) {
    throw std::runtime_error("Plant must be set before using WritePosition");
  }

  radian_t position_native;
  if (auto* arm = std::get_if<pdcsu::util::DefArmSys>(&plant_.value())) {
    position_native = arm->toNative(position);
  } else {
    throw std::runtime_error(
        "WritePosition<radian_t> requires DefArmSys plant");
  }

  if (soft_limits_.has_value()) {
    position_native = soft_limits_.value().LimitPosition(position_native);
  }

  auto response_pos =
      MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadPosition);
  radian_t current_position_native{response_pos};
  auto response_vel =
      MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadVelocity);
  radps_t current_velocity_native{response_vel};
  auto error = position_native.value() - current_position_native.value();

  double dc_target = genome_.gains.kP * error + genome_.gains.kI * 0.0 +
                     genome_.gains.kD * current_velocity_native.value() +
                     genome_.gains.kF * 0.0;

  MonkeyMaster::WriteDC(slot_id_, dc_target);
}

inline void HigherMotorController::WriteVelocityOnController(mps_t velocity) {
  if (!plant_.has_value()) {
    throw std::runtime_error(
        "Plant must be set before using WriteVelocityOnController");
  }

  radps_t velocity_native;
  if (auto* linear = std::get_if<pdcsu::util::DefLinearSys>(&plant_.value())) {
    velocity_native = linear->toNative(velocity);
  } else {
    throw std::runtime_error(
        "WriteVelocityOnController<mps_t> requires DefLinearSys plant");
  }

  if (soft_limits_.has_value()) {
    auto response_pos =
        MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadPosition);
    radian_t current_pos_native{response_pos};
    velocity_native =
        soft_limits_.value().LimitVelocity(velocity_native, current_pos_native);
  }
  MonkeyMaster::WriteVelocity(slot_id_, velocity_native);
}

inline void HigherMotorController::WriteVelocityOnController(radps_t velocity) {
  if (!plant_.has_value()) {
    throw std::runtime_error(
        "Plant must be set before using WriteVelocityOnController");
  }

  radps_t velocity_native;
  if (auto* arm = std::get_if<pdcsu::util::DefArmSys>(&plant_.value())) {
    velocity_native = arm->toNative(velocity);
  } else {
    throw std::runtime_error(
        "WriteVelocityOnController<radps_t> requires DefArmSys plant");
  }

  if (soft_limits_.has_value()) {
    auto response_pos =
        MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadPosition);
    radian_t current_pos_native{response_pos};
    velocity_native =
        soft_limits_.value().LimitVelocity(velocity_native, current_pos_native);
  }
  MonkeyMaster::WriteVelocity(slot_id_, velocity_native);
}

inline void HigherMotorController::WritePositionOnController(meter_t position) {
  if (!plant_.has_value()) {
    throw std::runtime_error(
        "Plant must be set before using WritePositionOnController");
  }

  radian_t position_native;
  if (auto* linear = std::get_if<pdcsu::util::DefLinearSys>(&plant_.value())) {
    position_native = linear->toNative(position);
  } else {
    throw std::runtime_error(
        "WritePositionOnController<meter_t> requires DefLinearSys plant");
  }

  if (soft_limits_.has_value()) {
    position_native = soft_limits_.value().LimitPosition(position_native);
  }
  MonkeyMaster::WritePosition(slot_id_, position_native);
}

inline void HigherMotorController::WritePositionOnController(
    radian_t position) {
  if (!plant_.has_value()) {
    throw std::runtime_error(
        "Plant must be set before using WritePositionOnController");
  }

  radian_t position_native;
  if (auto* arm = std::get_if<pdcsu::util::DefArmSys>(&plant_.value())) {
    position_native = arm->toNative(position);
  } else {
    throw std::runtime_error(
        "WritePositionOnController<radian_t> requires DefArmSys plant");
  }

  if (soft_limits_.has_value()) {
    position_native = soft_limits_.value().LimitPosition(position_native);
  }
  MonkeyMaster::WritePosition(slot_id_, position_native);
}

template <> inline mps_t HigherMotorController::GetVelocity<mps_t>() {
  if (!plant_.has_value()) {
    throw std::runtime_error("Plant must be set before using GetVelocity");
  }

  auto response =
      MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadVelocity);
  radps_t velocity_native{response};

  if (auto* linear = std::get_if<pdcsu::util::DefLinearSys>(&plant_.value())) {
    return linear->toReal(velocity_native);
  } else {
    throw std::runtime_error("GetVelocity<mps_t> requires DefLinearSys plant");
  }
}

template <> inline radps_t HigherMotorController::GetVelocity<radps_t>() {
  if (!plant_.has_value()) {
    throw std::runtime_error("Plant must be set before using GetVelocity");
  }

  auto response =
      MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadVelocity);
  radps_t velocity_native{response};

  if (auto* arm = std::get_if<pdcsu::util::DefArmSys>(&plant_.value())) {
    return arm->toReal(velocity_native);
  } else {
    throw std::runtime_error("GetVelocity<radps_t> requires DefArmSys plant");
  }
}

template <> inline meter_t HigherMotorController::GetPosition<meter_t>() {
  if (!plant_.has_value()) {
    throw std::runtime_error("Plant must be set before using GetPosition");
  }

  auto response =
      MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadPosition);
  radian_t position_native{response};

  if (auto* linear = std::get_if<pdcsu::util::DefLinearSys>(&plant_.value())) {
    return linear->toReal(position_native);
  } else {
    throw std::runtime_error(
        "GetPosition<meter_t> requires DefLinearSys plant");
  }
}

template <> inline radian_t HigherMotorController::GetPosition<radian_t>() {
  if (!plant_.has_value()) {
    throw std::runtime_error("Plant must be set before using GetPosition");
  }

  auto response =
      MonkeyMaster::Read(slot_id_, hardware::ReadType::kReadPosition);
  radian_t position_native{response};

  if (auto* arm = std::get_if<pdcsu::util::DefArmSys>(&plant_.value())) {
    return arm->toReal(position_native);
  } else {
    throw std::runtime_error("GetPosition<radian_t> requires DefArmSys plant");
  }
}

inline void HigherMotorController::SetPosition(meter_t position) {
  if (!plant_.has_value()) {
    throw std::runtime_error("Plant must be set before using SetPosition");
  }

  radian_t position_native;
  if (auto* linear = std::get_if<pdcsu::util::DefLinearSys>(&plant_.value())) {
    position_native = linear->toNative(position);
  } else {
    throw std::runtime_error(
        "SetPosition<meter_t> requires DefLinearSys plant");
  }

  MonkeyMaster::ZeroEncoder(slot_id_, position_native);
}

inline void HigherMotorController::SetPosition(radian_t position) {
  if (!plant_.has_value()) {
    throw std::runtime_error("Plant must be set before using SetPosition");
  }

  radian_t position_native;
  if (auto* arm = std::get_if<pdcsu::util::DefArmSys>(&plant_.value())) {
    position_native = arm->toNative(position);
  } else {
    throw std::runtime_error("SetPosition<radian_t> requires DefArmSys plant");
  }

  MonkeyMaster::ZeroEncoder(slot_id_, position_native);
}

}  // namespace funkit::control
