#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <optional>
#include <variant>

#include "funkit/control/HigherMotorController.h"
#include "funkit/robot/GenericSubsystem.h"
#include "pdcsu_units.h"

namespace funkit::robot::swerve {

struct SwerveModuleReadings {
  pdcsu::units::fps_t vel;
  pdcsu::units::foot_t drive_pos;
  pdcsu::units::degree_t steer_pos;
};

struct SwerveModuleOLControlTarget {
  pdcsu::units::fps_t drive;
  pdcsu::units::degree_t steer;
};

using SwerveModuleTarget = SwerveModuleOLControlTarget;  // TODO: remove

struct SwerveModuleUniqueConfig {
  std::string loc;

  int cancoder_id;
  int drive_id;
  int steer_id;

  pdcsu::units::ohm_t circuit_resistance;
};

using steer_conv_unit = pdcsu::units::scalar_t;
using drive_conv_unit =
    pdcsu::units::UnitCompound<pdcsu::units::foot_t, pdcsu::units::rotation_t>;

struct SwerveModuleCommonConfig {
  funkit::control::config::MotorConstructionParameters drive_params;
  funkit::control::config::MotorConstructionParameters steer_params;

  funkit::control::base::MotorMonkeyType motor_types;
  steer_conv_unit steer_reduction;
  drive_conv_unit drive_reduction;

  pdcsu::units::ohm_t avg_resistance;
  pdcsu::units::ohm_t circuit_resistance;

  pdcsu::units::nm_t steer_load_factor;

  pdcsu::util::DefLinearSys drive_plant;
  pdcsu::util::DefArmSys steer_plant;

  std::string_view bus = "";
};

/*
SwerveModuleSubsystem

A class representing a single swerve module. Controls a drive and steer motor
and a CANcoder. Meant to be constructed as a child subsystem of
DrivetrainSubsystem.
*/
class SwerveModuleSubsystem
    : public funkit::robot::GenericSubsystem<SwerveModuleReadings,
          SwerveModuleTarget> {
public:
  /*
  SwerveModuleSubsystem()

  Constructs a SwerveModuleSubsystem object with the given parameters. For use
  by DrivetrainSubsystem.
  */
  SwerveModuleSubsystem(Loggable& parent,
      SwerveModuleUniqueConfig unique_config,
      SwerveModuleCommonConfig common_config);

  void Setup() override;

  SwerveModuleTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  void SetCANCoderOffset();
  void SetCANCoderOffset(pdcsu::units::degree_t offset);

  void ZeroWithCANcoder();

  /*
  SetDriveGenome()

  Sets the drive motor genome. Must be called before Setup().
  */
  void SetDriveGenome(funkit::control::config::MotorGenome genome);

  /*
  SetSteerGenome()

  Sets the steer motor genome. Must be called before Setup().
  */
  void SetSteerGenome(funkit::control::config::MotorGenome genome);

  /*
  ModifyGenomes()

  Modifies the genome for the steer and drive motor controllers. Should be
  called after SwerveModuleSubsystem Setup, in DrivetrainSubsystem Setup.
  */
  void ModifySwerveGenome(funkit::control::config::MotorGenome drive_genome,
      funkit::control::config::MotorGenome steer_genome);

private:
  int last_rezero = 101;

  /*
  getMotorParams()

  Static helper function modifies the drive and steer motor parameters provided
  in the common configuration using the unique configuration.
  */
  static std::pair<funkit::control::config::MotorConstructionParameters,
      funkit::control::config::MotorConstructionParameters>
  getMotorParams(SwerveModuleUniqueConfig unique_config,
      SwerveModuleCommonConfig common_config);

  SwerveModuleReadings ReadFromHardware() override;

  void WriteToHardware(SwerveModuleTarget target) override;

  /*
  calculateSteerPosition()

  Calculates the direction for the steer motor, based on a normalized target.
  Also returns a boolean that represents the inversion of the drive motor.
  */
  std::pair<pdcsu::units::degree_t, bool> calculateSteerPosition(
      pdcsu::units::degree_t target_norm, pdcsu::units::degree_t current);

  pdcsu::units::ohm_t avg_resistance_;
  pdcsu::units::ohm_t circuit_resistance_;
  funkit::control::base::MotorMonkeyType motor_types_;

  funkit::control::config::MotorConstructionParameters drive_params_;
  funkit::control::config::MotorConstructionParameters steer_params_;

  funkit::control::HigherMotorController drive_;
  funkit::control::HigherMotorController steer_;

  std::optional<funkit::control::config::MotorGenome> drive_genome_;
  std::optional<funkit::control::config::MotorGenome> steer_genome_;

  pdcsu::util::DefLinearSys drive_plant_;
  pdcsu::util::DefArmSys steer_plant_;

  ctre::phoenix6::hardware::CANcoder cancoder_;

  pdcsu::units::fps_t max_speed_;

  pdcsu::units::nm_t steer_load_factor_;
};

}  // namespace funkit::robot::swerve