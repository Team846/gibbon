#pragma once

#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "funkit/base/Loggable.h"
#include "pdcsu_units.h"

namespace funkit::control::config {

/*
MotorConstructionParameters.

Contains all parameters necessary to construct a motor controller.

@param can_id: The CAN address of the motor controller.
@param bus: Use only if the motor controller is on the CTRE bus (CANivore).
@param max_wait_time: The maximum time before a control message times out.

*/
struct MotorConstructionParameters {
  int can_id;
  std::string_view bus = "";

  bool inverted{false};
  pdcsu::units::ms_t max_wait_time{20.0};
};

/*
Gains.

For PIDF control.
*/
struct Gains {
  double kP{0.0};
  double kI{0.0};
  double kD{0.0};
  double kF{0.0};
};

/*
MotorGenome.

Contains all configuration parameters for a motor controller.

@param motor_current_limit: The maximum current the hardware motor controller
will allow to the motor.
@param smart_current_limit: The maximum current funkit software will allow to
the motor.
@param voltage_compensation: Maximum voltage applied.
@param inverted: Whether the motor output is inverted.
@param brake_mode: Whether the motor is in brake mode.
*/
struct MotorGenome {
  pdcsu::units::amp_t motor_current_limit{40.0};
  pdcsu::units::amp_t smart_current_limit{40.0};
  pdcsu::units::volt_t voltage_compensation{12.0};

  bool brake_mode{true};

  Gains gains;
};

/*
SubsystemGenomeHelper.

Helper class for loading and saving genome preferences for subsystems.
*/
struct SubsystemGenomeHelper {
  static void CreateGainsPreferences(funkit::base::Loggable& sub,
      const std::string& gains_name, const Gains& backup) {
    sub.RegisterPreference(
        fmt::format("{}/{}", sub.name(), gains_name), backup.kP);
    sub.RegisterPreference(
        fmt::format("{}/{}", sub.name(), gains_name), backup.kI);
    sub.RegisterPreference(
        fmt::format("{}/{}", sub.name(), gains_name), backup.kD);
    sub.RegisterPreference(
        fmt::format("{}/{}", sub.name(), gains_name), backup.kF);
  }

  static Gains LoadGainsPreferences(
      funkit::base::Loggable& sub, const std::string& gains_name) {
    Gains gains;

    gains.kP = sub.GetPreferenceValue_double(
        fmt::format("{}/{}", sub.name(), gains_name));
    gains.kI = sub.GetPreferenceValue_double(
        fmt::format("{}/{}", sub.name(), gains_name));
    gains.kD = sub.GetPreferenceValue_double(
        fmt::format("{}/{}", sub.name(), gains_name));
    gains.kF = sub.GetPreferenceValue_double(
        fmt::format("{}/{}", sub.name(), gains_name));

    return gains;
  }

  static void CreateGenomePreferences(funkit::base::Loggable& sub,
      const std::string& genome_name, const MotorGenome& backup) {
    sub.RegisterPreference(fmt::format("{}/{}", sub.name(), genome_name),
        backup.motor_current_limit);
    sub.RegisterPreference(fmt::format("{}/{}", sub.name(), genome_name),
        backup.smart_current_limit);
    sub.RegisterPreference(fmt::format("{}/{}", sub.name(), genome_name),
        backup.voltage_compensation);
    sub.RegisterPreference(
        fmt::format("{}/{}", sub.name(), genome_name), backup.brake_mode);
    CreateGainsPreferences(
        sub, fmt::format("{}/gains", genome_name), backup.gains);
  }

  static MotorGenome LoadGenomePreferences(
      funkit::base::Loggable& sub, const std::string& genome_name) {
    MotorGenome genome;

    genome.motor_current_limit =
        sub.GetPreferenceValue_unit_type<pdcsu::units::amp_t>(
            fmt::format("{}/{}", sub.name(), genome_name));
    genome.smart_current_limit =
        sub.GetPreferenceValue_unit_type<pdcsu::units::amp_t>(
            fmt::format("{}/{}", sub.name(), genome_name));
    genome.voltage_compensation =
        sub.GetPreferenceValue_unit_type<pdcsu::units::volt_t>(
            fmt::format("{}/{}", sub.name(), genome_name));
    genome.brake_mode = sub.GetPreferenceValue_bool(
        fmt::format("{}/{}", sub.name(), genome_name));
    genome.gains =
        LoadGainsPreferences(sub, fmt::format("{}/gains", genome_name));

    return genome;
  }
};

/*
StatusFrame

Enum with types representing the status frame categories for common motor
controllers.
*/
enum StatusFrame {
  kPositionFrame,
  kVelocityFrame,
  kCurrentFrame,
  kFaultFrame,
  kSensorFrame,
  kAbsoluteFrame,
  kLeader
};

using StatusFrameSelections = std::vector<StatusFrame>;

}  // namespace funkit::control::config