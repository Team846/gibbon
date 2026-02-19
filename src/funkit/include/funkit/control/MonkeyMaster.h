#pragma once

#include <map>
#include <optional>
#include <queue>
#include <vector>

#include "funkit/base/Loggable.h"
#include "funkit/control/base/motor_control_base.h"
#include "funkit/control/base/motor_specs.h"
#include "funkit/control/calculators/CurrentTorqueCalculator.h"
#include "funkit/control/config/genome.h"
#include "funkit/control/hardware/IntermediateController.h"
#include "pdcsu_control.h"
#include "pdcsu_units.h"

#define CONTROLLER_REGISTRY_SIZE 64

namespace funkit::control {

/*
MonkeyMaster

A class that provides higher level management for all motor controllers. Manages
CAN utilization as well as power.
*/
class MonkeyMaster {
public:
  /*
  Setup()

  Sets up the preferences used by MonkeyMaster.
  */
  static void Setup();

  /*
  Tick()

  Updates all motor controllers. Should be called each loop.
  */
  static void Tick(bool disabled);

  static void SetNeutralMode(size_t slot_id, bool brake_mode);

  /*
  WriteMessages()

  Writes all messages in the message queue to the motor controllers. Also,
  drops redundant messages.

  If disabled is true, all written messages will be zeroed.
  */
  static void WriteMessages();

  /*
  ConstructController()

  Constructs a motor controller and returns a slot ID for it. This slot ID is
  used to refer to the motor controller in future calls.
  */
  static size_t ConstructController(funkit::control::base::MotorMonkeyType type,
      funkit::control::config::MotorConstructionParameters params,
      pdcsu::util::BasePlant plant, config::MotorGenome genome);

  /*
  EnableStatusFrames()

  Enables specific status frames for a motor controller. Disables all others.
  */
  static void EnableStatusFrames(size_t slot_id,
      config::StatusFrameSelections frames,
      pdcsu::units::ms_t faults_ms = pdcsu::units::ms_t{20},
      pdcsu::units::ms_t velocity_ms = pdcsu::units::ms_t{20},
      pdcsu::units::ms_t encoder_position_ms = pdcsu::units::ms_t{20},
      pdcsu::units::ms_t analog_position_ms = pdcsu::units::ms_t{20});

  static void OverrideStatusFramePeriod(size_t slot_id,
      funkit::control::config::StatusFrame frame, pdcsu::units::ms_t period);

  /*
  GetBatteryVoltage()

  Returns the battery voltage.
  */
  static pdcsu::units::volt_t GetBatteryVoltage();

  /*
  SetLoad()

  Sets the load on a motor controller. This is used for calculations.
  */
  static void SetLoad(size_t slot_id, pdcsu::units::nm_t load);

  static void SetGenome(
      size_t slot_id, config::MotorGenome genome, bool force_set = false);

  static void WriteDC(size_t slot_id, double duty_cycle);
  /*
  WriteVelocity()

  Writes a velocity setpoint to the motor controller. PID calculations performed
  onboard the motor controller.
  */
  static void WriteVelocity(size_t slot_id, pdcsu::units::radps_t velocity);
  /*
  WritePosition()

  Writes a position setpoint to the motor controller. PID calculations
  performed onboard the motor controller.
  */
  static void WritePosition(size_t slot_id, pdcsu::units::radian_t position);

  static hardware::ReadResponse Read(size_t slot_id, hardware::ReadType type);

  static void SpecialConfigure(
      size_t slot_id, hardware::SpecialConfigureType type);

  static void SetSoftLimits(size_t slot_id,
      pdcsu::units::radian_t forward_limit,
      pdcsu::units::radian_t reverse_limit);

  static void ZeroEncoder(size_t slot_id, pdcsu::units::radian_t position);

  static std::string_view parseError(
      funkit::control::hardware::ControllerErrorCodes err);

  static bool VerifyConnected();
  static bool VerifyConnected(size_t slot_id);

  static void CheckForResets();

private:
  static funkit::base::Loggable loggable_;

  static size_t slot_counter_;
  static std::map<size_t, funkit::control::base::MotorMonkeyType>
      slot_id_to_type_;
  static std::map<size_t, bool> slot_id_to_sim_;

  static funkit::control::hardware::IntermediateController*
      controller_registry[CONTROLLER_REGISTRY_SIZE];

  static config::MotorGenome genome_registry[CONTROLLER_REGISTRY_SIZE];
  static pdcsu::units::nm_t load_registry[CONTROLLER_REGISTRY_SIZE];

  static bool reset_registry[CONTROLLER_REGISTRY_SIZE];
  static size_t consecutive_reset_counts[CONTROLLER_REGISTRY_SIZE];
  static size_t reset_skip_counters[CONTROLLER_REGISTRY_SIZE];

  static std::optional<pdcsu::util::BasePlant>
      plant_registry[CONTROLLER_REGISTRY_SIZE];

  static pdcsu::units::volt_t battery_voltage;

  struct MotorMessage {
    enum class Type { DC, Position, Velocity };
    size_t slot_id;
    Type type;
    std::variant<double, pdcsu::units::radian_t, pdcsu::units::radps_t> value;
  };

  static std::queue<MotorMessage> control_messages;
};

}  // namespace funkit::control
