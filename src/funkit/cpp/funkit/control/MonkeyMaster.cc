#include "funkit/control/MonkeyMaster.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <units/current.h>
#include <units/voltage.h>

#include <iostream>
#include <string_view>
#include <vector>

#include "funkit/control/SupremeLimiter.h"
#include "funkit/control/hardware/SparkMXFX_interm.h"
#include "funkit/control/hardware/TalonFX_interm.h"
#include "funkit/control/hardware/simulation/SIMLEVEL.h"
#include "funkit/control/hardware/simulation/VirtualMonkey.h"
#include "funkit/math/collection.h"
#include "pdcsu_control.h"
#include "pdcsu_units.h"

namespace funkit::control {

#define CHECK_SLOT_ID()                                                       \
  if (controller_registry[slot_id] == nullptr)                                \
    throw std::runtime_error(                                                 \
        "Invalid MonkeyMaster slot ID: " + std::to_string(slot_id) + " in " + \
        "[" + __func__ + "]");

#define LOG_IF_ERROR(action_name)                                          \
  {                                                                        \
    hardware::ControllerErrorCodes err =                                   \
        controller_registry[slot_id]->GetLastErrorCode();                  \
    if (err != hardware::ControllerErrorCodes::kAllOK) {                   \
      loggable_.Error("Error [{}] completing action [{}] for slot ID {}.", \
          parseError(err), action_name, slot_id);                          \
    }                                                                      \
  }

#define NUM_RETRIES 5
#define INITIAL_RETRY_DELAY_MS 10

#define SMART_RETRY(action, action_name)                                    \
  for (int i = 0; i < NUM_RETRIES; i++) {                                   \
    action;                                                                 \
    hardware::ControllerErrorCodes err =                                    \
        controller_registry[slot_id]->GetLastErrorCode();                   \
    if (err == hardware::ControllerErrorCodes::kAllOK)                      \
      break;                                                                \
    else if (i == NUM_RETRIES - 1) {                                        \
      loggable_.Error("Failed [{}] for slot ID {}.", action_name, slot_id); \
    } else {                                                                \
      loggable_.Warn(                                                       \
          "Error [{}] while attempting [{}] for slot ID {}. Retrying...",   \
          parseError(err), action_name, slot_id);                           \
      std::this_thread::sleep_for(                                          \
          std::chrono::milliseconds(INITIAL_RETRY_DELAY_MS * (1 << i)));    \
    }                                                                       \
  }

funkit::base::Loggable MonkeyMaster::loggable_{"MonkeyMaster"};

size_t MonkeyMaster::slot_counter_{0};
std::map<size_t, funkit::control::base::MotorMonkeyType>
    MonkeyMaster::slot_id_to_type_{};
std::map<size_t, bool> MonkeyMaster::slot_id_to_sim_{};

funkit::control::hardware::IntermediateController*
    MonkeyMaster::controller_registry[CONTROLLER_REGISTRY_SIZE]{};

config::MotorGenome MonkeyMaster::genome_registry[CONTROLLER_REGISTRY_SIZE]{};

pdcsu::units::nm_t MonkeyMaster::load_registry[CONTROLLER_REGISTRY_SIZE]{};

std::optional<pdcsu::util::BasePlant>
    MonkeyMaster::plant_registry[CONTROLLER_REGISTRY_SIZE]{};

pdcsu::units::volt_t MonkeyMaster::battery_voltage{pdcsu::units::volt_t{0}};

std::queue<MonkeyMaster::MotorMessage> MonkeyMaster::control_messages{};

void MonkeyMaster::Setup() {}

void MonkeyMaster::Tick(bool disabled) {
  battery_voltage = pdcsu::units::volt_t{
      frc::RobotController::GetBatteryVoltage().to<double>()};

  if (disabled) {
    if (!frc::RobotBase::IsSimulation()) {
      WriteMessages();
      MonkeyMaster::loggable_.Graph("LoggableTick", -battery_voltage);
      return;
    }
  }
  MonkeyMaster::loggable_.Graph("LoggableTick", battery_voltage);

  WriteMessages();

  for (size_t i = 0; i < CONTROLLER_REGISTRY_SIZE; i++) {
    if (controller_registry[i] != nullptr) {
      if (slot_id_to_sim_[i]) {
        simulation::VirtualMonkey* sim =
            dynamic_cast<simulation::VirtualMonkey*>(controller_registry[i]);
        sim->SetBatteryVoltage(battery_voltage);
        sim->SetLoad(load_registry[i]);
        if (disabled) {
          base::ControlRequest zero = 0.0;
          sim->Write(zero);
        }
        sim->Tick();
      } else {
        controller_registry[i]->Tick();
      }
    }
  }
}

void MonkeyMaster::SetNeutralMode(size_t slot_id, bool brake_mode) {
  CHECK_SLOT_ID();
  genome_registry[slot_id].brake_mode = brake_mode;
  SMART_RETRY(controller_registry[slot_id]->SetGenome(genome_registry[slot_id]),
      "SetNeutralMode");
  LOG_IF_ERROR("SetNeutralMode");
}

void MonkeyMaster::WriteMessages() {
  std::vector<MotorMessage> messages_to_process;
  messages_to_process.reserve(control_messages.size());

  while (!control_messages.empty()) {
    messages_to_process.push_back(control_messages.front());
    control_messages.pop();
  }

  std::vector<PerDeviceInformation> per_device_information;
  for (const auto& msg : messages_to_process) {
    size_t slot_id = msg.slot_id;
    auto* controller = controller_registry[slot_id];

    double DC = 0.0;
    bool is_limitable = false;

    if (msg.type == MotorMessage::Type::DC) {
      DC = std::clamp(std::get<double>(msg.value), -1.0, 1.0);
      is_limitable = true;
    } else if (msg.type == MotorMessage::Type::Position) {
      bool isCTRE = control::base::MotorMonkeyTypeHelper::is_talon_fx(
          slot_id_to_type_[slot_id]);

      radian_t position = std::get<pdcsu::units::radian_t>(msg.value);
      radian_t current_position{
          controller->Read(hardware::ReadType::kReadPosition)};
      radps_t current_velocity{
          controller->Read(hardware::ReadType::kReadVelocity)};

      rotation_t error = position - current_position;

      auto gains = genome_registry[slot_id].gains;
      if (isCTRE) {
        DC = std::clamp(
            (error.value() * gains.kP +
                UnitDivision<rotation_t, second_t>(current_velocity).value() *
                    gains.kD) /
                12.0,
            -1.0, 1.0);
      } else {
        DC = std::clamp(
            error.value() * gains.kP +
                UnitDivision<rotation_t, ms_t>(current_velocity).value() *
                    gains.kD,
            -1.0, 1.0);
      }
    }
    if (plant_registry[slot_id].has_value()) {
      per_device_information.push_back({slot_id, *plant_registry[slot_id],
          radps_t{controller->Read(hardware::ReadType::kReadVelocity)}, DC,
          is_limitable});
    }
  }

  auto limited_dcs =
      SupremeLimiter::Limit(per_device_information, battery_voltage);

  for (const auto& msg : messages_to_process) {
    size_t slot_id = msg.slot_id;
    auto* controller = controller_registry[slot_id];

    switch (msg.type) {
    case MotorMessage::Type::DC: {
      base::ControlRequest cr = limited_dcs[slot_id];
      if (!controller->IsDuplicateControlMessage(cr) ||
          controller->GetLastErrorCode() !=
              funkit::control::hardware::ControllerErrorCodes::kAllOK) {
        controller->Write(cr);
        LOG_IF_ERROR("WriteDC");
      }
      break;
    }
    case MotorMessage::Type::Position: {
      auto pos = std::get<pdcsu::units::radian_t>(msg.value);
      base::ControlRequest cr = pos;
      if (!controller->IsDuplicateControlMessage(cr) ||
          controller->GetLastErrorCode() !=
              funkit::control::hardware::ControllerErrorCodes::kAllOK) {
        controller->Write(cr);
        LOG_IF_ERROR("WritePosition");
      }
      break;
    }
    case MotorMessage::Type::Velocity: {
      auto vel = std::get<pdcsu::units::radps_t>(msg.value);
      base::ControlRequest cr = vel;
      if (!controller->IsDuplicateControlMessage(cr) ||
          controller->GetLastErrorCode() !=
              funkit::control::hardware::ControllerErrorCodes::kAllOK) {
        controller->Write(cr);
        LOG_IF_ERROR("WriteVelocity");
      }
      break;
    }
    }
  }
}

bool MonkeyMaster::VerifyConnected() {
  for (size_t i = 0; i < CONTROLLER_REGISTRY_SIZE; i++) {
    if (controller_registry[i] != nullptr) {
      if (!controller_registry[i]->VerifyConnected()) { return false; }
    }
  }
  return true;
}

bool MonkeyMaster::VerifyConnected(size_t slot_id) {
  if (slot_id >= CONTROLLER_REGISTRY_SIZE) return false;
  if (controller_registry[slot_id] == nullptr) return false;
  return controller_registry[slot_id]->VerifyConnected();
}

size_t MonkeyMaster::ConstructController(
    funkit::control::base::MotorMonkeyType type,
    funkit::control::config::MotorConstructionParameters params,
    pdcsu::util::BasePlant plant, config::MotorGenome genome) {
  slot_counter_++;

  size_t slot_id = slot_counter_;
  slot_id_to_type_[slot_id] = type;
  slot_id_to_sim_[slot_id] = false;

  funkit::control::hardware::IntermediateController* this_controller = nullptr;

  if (frc::RobotBase::IsSimulation() &&
      MOTOR_SIM_LEVEL == MOTOR_SIM_LEVEL_SIM_PHYSICS) {
    std::cout << "Constructing simulation controller" << std::endl;
    loggable_.Log(
        "Constructing physics simulation controller for slot ID {}.", slot_id);
    slot_id_to_sim_[slot_id] = true;
    this_controller = controller_registry[slot_id] =
        new funkit::control::simulation::VirtualMonkey{
            funkit::control::base::MotorSpecificationPresets::get(type),
            pdcsu::units::ohm_t{0.01}, pdcsu::units::kgm2_t{0.0}, 0.0};
  } else if (funkit::control::base::MotorMonkeyTypeHelper::is_talon_fx(type)) {
    this_controller = controller_registry[slot_id] =
        new funkit::control::hardware::TalonFX_interm{params.can_id, params.bus,
            pdcsu::units::ms_t{params.max_wait_time.value()}, params.inverted};

  } else if (funkit::control::base::MotorMonkeyTypeHelper::is_spark_max(type)) {
    this_controller = controller_registry[slot_id] =
        new funkit::control::hardware::SparkMAX_interm{params.can_id,
            pdcsu::units::ms_t{params.max_wait_time.value()}, type,
            params.inverted};
  } else if (funkit::control::base::MotorMonkeyTypeHelper::is_spark_flex(
                 type)) {
    this_controller = controller_registry[slot_id] =
        new funkit::control::hardware::SparkFLEX_interm{params.can_id,
            pdcsu::units::ms_t{params.max_wait_time.value()}, params.inverted};
  } else {
    throw std::runtime_error("Invalid MotorMonkeyType [" +
                             std::to_string((int)type) +
                             "]: not constructing controller");
  }

  if (this_controller == nullptr) { return slot_id; }

  plant_registry[slot_id] = plant;

  genome_registry[slot_id] = genome;

  SMART_RETRY(this_controller->SetGenome(genome), "SetGenome");
  LOG_IF_ERROR("SetGenome");

  return slot_id;
}

void MonkeyMaster::EnableStatusFrames(size_t slot_id,
    config::StatusFrameSelections frames, pdcsu::units::ms_t faults_ms,
    pdcsu::units::ms_t velocity_ms, pdcsu::units::ms_t encoder_position_ms,
    pdcsu::units::ms_t analog_position_ms) {
  CHECK_SLOT_ID();

  SMART_RETRY(
      controller_registry[slot_id]->EnableStatusFrames(frames, faults_ms,
          velocity_ms, encoder_position_ms, analog_position_ms),
      "EnableStatusFrames");
  LOG_IF_ERROR("EnableStatusFrames");
}

void MonkeyMaster::OverrideStatusFramePeriod(size_t slot_id,
    funkit::control::config::StatusFrame frame, pdcsu::units::ms_t period) {
  CHECK_SLOT_ID();
  SMART_RETRY(
      controller_registry[slot_id]->OverrideStatusFramePeriod(frame, period),
      "OverrideStatusFramePeriod");
  LOG_IF_ERROR("OverrideStatusFramePeriod");
}

pdcsu::units::volt_t MonkeyMaster::GetBatteryVoltage() {
  return battery_voltage;
}

void MonkeyMaster::SetLoad(size_t slot_id, pdcsu::units::nm_t load) {
  CHECK_SLOT_ID();

  load_registry[slot_id] = load;
}

void MonkeyMaster::SetGenome(
    size_t slot_id, config::MotorGenome genome, bool force_set) {
  CHECK_SLOT_ID();

  genome_registry[slot_id] = genome;

  SMART_RETRY(
      controller_registry[slot_id]->SetGenome(genome, force_set), "SetGenome");
  LOG_IF_ERROR("SetGenome");
}

void MonkeyMaster::WriteDC(size_t slot_id, double duty_cycle) {
  CHECK_SLOT_ID();

  control_messages.push({slot_id, MotorMessage::Type::DC, duty_cycle});
}

void MonkeyMaster::WriteVelocity(
    size_t slot_id, pdcsu::units::radps_t velocity) {
  CHECK_SLOT_ID();

  control_messages.push({slot_id, MotorMessage::Type::Velocity, velocity});
}

void MonkeyMaster::WritePosition(
    size_t slot_id, pdcsu::units::radian_t position) {
  CHECK_SLOT_ID();

  control_messages.push({slot_id, MotorMessage::Type::Position, position});
}

hardware::ReadResponse MonkeyMaster::Read(
    size_t slot_id, hardware::ReadType type) {
  CHECK_SLOT_ID();

  return controller_registry[slot_id]->Read(type);
}

void MonkeyMaster::SpecialConfigure(
    size_t slot_id, hardware::SpecialConfigureType type) {
  CHECK_SLOT_ID();

  SMART_RETRY(
      controller_registry[slot_id]->SpecialConfigure(type), "SpecialConfigure");
  LOG_IF_ERROR("SpecialConfigure");
}

void MonkeyMaster::SetSoftLimits(size_t slot_id,
    pdcsu::units::radian_t forward_limit,
    pdcsu::units::radian_t reverse_limit) {
  CHECK_SLOT_ID();

  SMART_RETRY(
      controller_registry[slot_id]->SetSoftLimits(forward_limit, reverse_limit),
      "SetSoftLimits");
  LOG_IF_ERROR("SetSoftLimits");
}

void MonkeyMaster::ZeroEncoder(
    size_t slot_id, pdcsu::units::radian_t position) {
  CHECK_SLOT_ID();

  SMART_RETRY(
      controller_registry[slot_id]->ZeroEncoder(position), "ZeroEncoder");
  LOG_IF_ERROR("ZeroEncoder");
}

std::string_view MonkeyMaster::parseError(
    funkit::control::hardware::ControllerErrorCodes err) {
  switch (err) {
  case funkit::control::hardware::ControllerErrorCodes::kAllOK: return "All OK";
  case funkit::control::hardware::ControllerErrorCodes::kInvalidCANID:
    return "Invalid CAN ID";
  case funkit::control::hardware::ControllerErrorCodes::kVersionMismatch:
    return "Version Mismatch";
  case funkit::control::hardware::ControllerErrorCodes::kHALError:
    return "HAL Error";
  case funkit::control::hardware::ControllerErrorCodes::kFollowingError:
    return "Following Error";
  case funkit::control::hardware::ControllerErrorCodes::kCANDisconnected:
    return "CAN Disconnected";
  case funkit::control::hardware::ControllerErrorCodes::kCANMessageStale:
    return "CAN Message Stale";
  case funkit::control::hardware::ControllerErrorCodes::kDeviceDisconnected:
    return "Device Disconnected";
  case funkit::control::hardware::ControllerErrorCodes::kConfigFailed:
    return "Config Failed";
  case funkit::control::hardware::ControllerErrorCodes::kTimeout:
    return "Timeout";
  case funkit::control::hardware::ControllerErrorCodes::kWarning:
    return "Unknown Warning";
  case funkit::control::hardware::ControllerErrorCodes::kError:
    return "Unknown Error";
  default: return "Unknown";
  }
}

void MonkeyMaster::CheckForResets() {
  for (size_t i = 1; i <= slot_counter_; i++) {
    if (controller_registry[i] != nullptr) {
      if (controller_registry[i]->Read(hardware::ReadType::kRestFault) > 0.5) {
        loggable_.Warn(
            "Reset detected on Motor Controller Slot ID {}. Reconfiguring...",
            i);
        SetGenome(i, genome_registry[i], true);
      }
    }
  }
}

}  // namespace funkit::control
