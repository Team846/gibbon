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

class GenericRobot : public frc::RobotBase, public funkit::base::Loggable {
public:
  static constexpr pdcsu::units::second_t kPeriod{0.01};  // 100Hz (10ms)

  GenericRobot(GenericRobotContainer* container);

  ~GenericRobot() override;

  void StartCompetition() override final;
  void EndCompetition() override final;

  virtual void OnInitialize() = 0;
  virtual void OnEnable() = 0;

  virtual void OnDisable() = 0;

  virtual void OnPeriodic() = 0;

  virtual void InitTeleop() = 0;
  virtual void InitTest() = 0;

  void VerifyHardware();

  void AddAuto(std::string name, frc2::Command* command);
  void AddDefaultAuto(std::string name, frc2::Command* command);

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
};

}  // namespace funkit::robot
