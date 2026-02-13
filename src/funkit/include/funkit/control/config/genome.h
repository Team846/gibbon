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
FollowerConfig.

For configuring a motor controller as a follower.
*/
struct FollowerConfig {
  int leader_CAN_id{-1};
  bool inverted{false};
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

  FollowerConfig follower_config;
};

/*
SubsystemGenomeHelper.

Helper class for loading and saving genome preferences for subsystems.
*/
struct SubsystemGenomeHelper {
  static void CreateGainsPreferences(funkit::base::Loggable& sub,
      const Gains& backup, const std::string& genome_name) {
    sub.RegisterPreference(
        fmt::format("{}/{}", genome_name, "gains/kP"), backup.kP);
    sub.RegisterPreference(
        fmt::format("{}/{}", genome_name, "gains/kI"), backup.kI);
    sub.RegisterPreference(
        fmt::format("{}/{}", genome_name, "gains/kD"), backup.kD);
    sub.RegisterPreference(
        fmt::format("{}/{}", genome_name, "gains/kF"), backup.kF);
  }

  static Gains LoadGainsPreferences(
      funkit::base::Loggable& sub, const std::string& genome_name) {
    Gains gains;

    gains.kP = sub.GetPreferenceValue_double(
        fmt::format("{}/{}", genome_name, "gains/kP"));
    gains.kI = sub.GetPreferenceValue_double(
        fmt::format("{}/{}", genome_name, "gains/kI"));
    gains.kD = sub.GetPreferenceValue_double(
        fmt::format("{}/{}", genome_name, "gains/kD"));
    gains.kF = sub.GetPreferenceValue_double(
        fmt::format("{}/{}", genome_name, "gains/kF"));

    return gains;
  }

  static void CreateGenomePreferences(funkit::base::Loggable& sub,
      const std::string& genome_name, const MotorGenome& backup) {
    sub.RegisterPreference<amp_t>(
        fmt::format("{}/{}", genome_name, "motor_current_limit"),
        backup.motor_current_limit);
    sub.RegisterPreference<amp_t>(
        fmt::format("{}/{}", genome_name, "smart_current_limit"),
        backup.smart_current_limit);
    sub.RegisterPreference<volt_t>(
        fmt::format("{}/{}", genome_name, "voltage_compensation"),
        backup.voltage_compensation);
    sub.RegisterPreference(
        fmt::format("{}/{}", genome_name, "brake_mode"), backup.brake_mode);
    CreateGainsPreferences(sub, backup.gains, genome_name);
  }

  static MotorGenome LoadGenomePreferences(
      funkit::base::Loggable& sub, const std::string& genome_name) {
    MotorGenome genome;

    genome.motor_current_limit = sub.GetPreferenceValue_unit_type<amp_t>(
        fmt::format("{}/{}", genome_name, "motor_current_limit"));
    genome.smart_current_limit = sub.GetPreferenceValue_unit_type<amp_t>(
        fmt::format("{}/{}", genome_name, "smart_current_limit"));
    genome.voltage_compensation = sub.GetPreferenceValue_unit_type<volt_t>(
        fmt::format("{}/{}", genome_name, "voltage_compensation"));
    genome.brake_mode = sub.GetPreferenceValue_bool(
        fmt::format("{}/{}", genome_name, "brake_mode"));
    genome.gains = LoadGainsPreferences(sub, genome_name);

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