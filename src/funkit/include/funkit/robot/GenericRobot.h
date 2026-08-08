#pragma once

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <hal/Types.h>
#include <units/time.h>

#include "funkit/robot/GenericRobotContainer.h"

namespace frc2 {
class Command;
}

namespace funkit::robot {

enum Mode { kNone, kDisabled, kAutonomous, kTeleop, kTest };

/**
 * GenericRobot
 *
 * A class that inherits from RobotBase and Loggable, helping provide logging
 * utilities beyond WPILibs RobotBase.
 */
class GenericRobot : public frc::RobotBase, public funkit::base::Loggable {
public:
  static constexpr pdcsu::units::second_t kPeriod{0.01};  // 100Hz (10ms)

  GenericRobot(GenericRobotContainer* container);

  ~GenericRobot() override;

  /**
   * StartCompetition()
   *
   * Initializes robot services and enters the competition lifecycle loop
   */
  void StartCompetition() override final;

  /**
   * EndCompetition()
   *
   * Logs the competition end.
   */
  void EndCompetition() override final;

  virtual void OnInitialize() = 0;
  virtual void OnEnable() = 0;

  virtual void OnDisable() = 0;

  virtual void OnPeriodic() = 0;

  virtual void InitTeleop() = 0;
  virtual void InitTest() = 0;

  //
  virtual void ClearDefaultCommands() = 0;

  // Verifies all hardware is connected
  void VerifyHardware();

  /**
   * AddAuto()
   *
   * Adds an autonomous sequence option for GenericRobot to run through the
   * SmartDashboard
   * @param name - the name to reference the auto by
   * @param command - a pointer to the auto command
   */
  void AddAuto(std::string name, frc2::Command* command);

  /**
   * AddDefaultAuto()
   *
   * Adds a default autonomous sequence for GenericRobot to run through the
   * SmartDashboard unless specified otherwise
   * @param name - the name to reference the auto by
   * @param command - a pointer to the auto command
   */
  void AddDefaultAuto(std::string name, frc2::Command* command);

  // Getter method for a selected auto
  static std::string GetSelectedAuto() { return auto_chooser_.GetSelected(); }

private:
  hal::Handle<HAL_NotifierHandle> notifier_;
  pdcsu::units::second_t next_loop_time_;

  Mode last_mode_;

private:
  GenericRobotContainer* generic_robot_container_;

  frc2::Command* auto_command_ = nullptr;
  static frc::SendableChooser<std::string> auto_chooser_;
  std::unordered_map<std::string, frc2::Command*> autos_;

  int update_tick_counter_ = 0;

  int cached_update_tick_1_ = 100;
  int cached_update_tick_2_ = 201;
  int cached_update_reset_tick_ = 200;
};

}  // namespace funkit::robot
