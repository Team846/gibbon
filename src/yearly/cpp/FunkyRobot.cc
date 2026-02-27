#include "FunkyRobot.h"

#include <cameraserver/CameraServer.h>
#include <frc/DSControlWord.h>
#include <frc/Filesystem.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>
#include <hal/Notifier.h>
#include <networktables/NetworkTableInstance.h>

#include "autos/auton_seqs.h"
#include "calculators/AllianceShiftCalculator.h"
#include "calculators/ShootingCalculator.h"
#include "commands/teleop/drive_command.h"
#include "commands/teleop/hoptake_command.h"
#include "commands/teleop/scorer_command.h"
#include "control_triggers.h"
#include "funkit/wpilib/NTAction.h"
#include "rsighandler.h"
#include "subsystems/hardware/leds_logic.h"

FunkyRobot::FunkyRobot() : GenericRobot{&container_} {
  RegisterPreference("num_coasting_loops", 1000);
  RegisterPreference("homing_flash_loops", 50);

  // std::thread visionThread{[&]() {
  //   VisionThread(&container_);
  // }};
  // visionThread.detach();
}

void FunkyRobot::OnInitialize() {
  ShootingCalculator::Setup();

  ADD_AUTO_VARIANTS(CS2Auto, "CS2");

  // Add dashboard buttons
  frc::SmartDashboard::PutData("set_cancoder_offsets",
      new funkit::wpilib::NTAction(
          [this] { container_.drivetrain_.SetCANCoderOffsets(); }));
  frc::SmartDashboard::PutData(
      "zero_bearing", new funkit::wpilib::NTAction(
                          [this] { container_.drivetrain_.ZeroBearing(); }));

  frc::SmartDashboard::PutData(
      "zero_odometry", new funkit::wpilib::NTAction([this] {
        container_.drivetrain_.SetPosition({inch_t{0}, inch_t{0}});
      }));

  frc::SmartDashboard::PutData("zero_turret_encoders",
      new funkit::wpilib::NTAction(
          [this] { container_.scorer_ss_.turret.ZeroEncoders(); }));

  frc::SmartDashboard::PutData("zero_turret_with_CRT",
      new funkit::wpilib::NTAction(
          [this] { container_.scorer_ss_.turret.ZeroWithCRT(); }));

  // Add path recording controls
  frc::SmartDashboard::PutData(
      "start_path_recording", new funkit::wpilib::NTAction([this] {
        auto timestamp =
            std::chrono::system_clock::now().time_since_epoch().count();
        std::string filename = "path_" + std::to_string(timestamp);
        container_.drivetrain_.StartPathRecording(filename);
        Log("Started recording path data to {}.csv", filename);
      }));

  frc::SmartDashboard::PutData(
      "stop_path_recording", new funkit::wpilib::NTAction([this] {
        bool success = container_.drivetrain_.StopPathRecording();
        if (success) {
          Log("Successfully stopped recording path data");
        } else {
          Warn(
              "Failed to stop recording path data or no recording in progress");
        }
      }));
}

void FunkyRobot::OnEnable() {
  // Start path recording
  // if (container_.drivetrain_.IsPathRecording()) {
  //   auto now = std::chrono::system_clock::now();
  //   auto time_t_now = std::chrono::system_clock::to_time_t(now);

  //   // Format time as month-day-year-hour-minute-second
  //   std::tm* tm_now = std::localtime(&time_t_now);

  //   char time_buffer[64];
  //   std::strftime(
  //       time_buffer, sizeof(time_buffer), "%m-%d-%Y_%H-%M-%S", tm_now);
  //   std::string filename = "pathlogs_" + std::string(time_buffer);

  // container_.drivetrain_.StartPathRecording(filename);
  // Log("Started recording auto path data to {}.csv", filename);
  // }
}

void FunkyRobot::OnDisable() {
  // Stop path recording
  // bool success = container_.drivetrain_.StopPathRecording();
  // if (success) {
  //   Log("Successfully stopped recording path data");
  // } else {
  //   Warn("Failed to stop recording path data or no recording in progress");
  // }
}

void FunkyRobot::InitTeleop() {
  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});
  container_.scorer_ss_.SetDefaultCommand(ScorerCommand{container_});
  container_.hoptake_ss_.SetDefaultCommand(HoptakeCommand{container_});
  ControlTriggerInitializer::InitTeleopTriggers(container_);
}

void FunkyRobot::ClearDefaultCommands() {
  container_.drivetrain_.RemoveDefaultCommand();
  container_.scorer_ss_.RemoveDefaultCommand();
  container_.hoptake_ss_.RemoveDefaultCommand();
}

void FunkyRobot::OnPeriodic() {
  ShootingCalculator::Calculate(&container_);

  if (frc::RobotBase::IsSimulation()) {
    auto instance = nt::NetworkTableInstance::GetDefault();
    auto funkyFMSTable = instance.GetTable("FunkyFMS");
    auto controlModeEntry = funkyFMSTable->GetEntry("controlMode");

    if (controlModeEntry.Exists()) {
      int modeValue = static_cast<int>(controlModeEntry.GetInteger(0));

      switch (modeValue) {
      case 0:
        frc::sim::DriverStationSim::SetEnabled(false);
        frc::sim::DriverStationSim::SetAutonomous(false);
        frc::sim::DriverStationSim::SetTest(false);
        break;
      case 1:
        frc::sim::DriverStationSim::SetEnabled(true);
        frc::sim::DriverStationSim::SetAutonomous(false);
        frc::sim::DriverStationSim::SetTest(false);
        break;
      case 2:
        frc::sim::DriverStationSim::SetEnabled(true);
        frc::sim::DriverStationSim::SetAutonomous(true);
        frc::sim::DriverStationSim::SetTest(false);
        break;
      case 3:
        frc::sim::DriverStationSim::SetEnabled(true);
        frc::sim::DriverStationSim::SetAutonomous(false);
        frc::sim::DriverStationSim::SetTest(true);
        break;
      }
      frc::sim::DriverStationSim::NotifyNewData();
    }
  }

  if (container_.scorer_ss_.turret.is_initialized() &&
      container_.drivetrain_.is_initialized()) {
    container_.drivetrain_.SetFieldObjectPose("turret",
        container_.drivetrain_.GetReadings().estimated_pose.position,
        container_.scorer_ss_.turret.GetReadings().pos_ +
            container_.drivetrain_.GetReadings().pose.bearing);
  }

  if (!gyro_switch_.Get() && !IsEnabled()) {
    container_.drivetrain_.SetBearing(degree_t{0});
    homing_count_gyro = GetPreferenceValue_int("homing_flash_loops");
  }

  if (!home_switch_.Get() && !IsEnabled()) {
    homing_count_ = GetPreferenceValue_int("homing_flash_loops");
  }

  if (coast_count_ > 0) coast_count_--;
  if (coast_count_ == 1 || coast_count_ == 7) {}
  if (!coast_switch_.Get() && !IsEnabled()) {
    coast_count_ = GetPreferenceValue_int("num_coasting_loops");
  }

  if (homing_count_ > 0) homing_count_--;
  if (homing_count_gyro > 0) homing_count_gyro--;

  bool isDisabled = frc::DriverStation::IsDisabled();

  if (!isDisabled) {
    auto shift_data = AllianceShiftCalculator::Calculate();
    Graph("game_data/shift_active", shift_data.our_hub_active);
    Graph("game_data/active_first", shift_data.won_auto);
    Graph("game_data/shift", shift_data.shift);
    Graph("game_data/until_flip", shift_data.until_flip);
    Graph("game_data/our_time_left", shift_data.our_time_left);
  }

  if (homing_count_ > 0 && isDisabled)
    LEDsLogic::SetLEDsState(&container_, kLEDsHoming);
  else if (homing_count_gyro > 0 && isDisabled)
    LEDsLogic::SetLEDsState(&container_, kLEDsHomingGyro);
  else if (coast_count_ > 0 && isDisabled)
    LEDsLogic::CoastingLEDs(&container_,
        (1.0 * coast_count_) / GetPreferenceValue_int("num_coasting_loops"));
  else if (container_.scorer_ss_.GetReadings().will_make_shot &&
           container_.drivetrain_.variance < 16.0)
    LEDsLogic::SetLEDsState(&container_, kLEDsSequencing);
  else
    LEDsLogic::UpdateLEDs(&container_);
}

void FunkyRobot::InitTest() {
  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});
}

#ifndef RUNNING_FRC_TESTS
int main() {
  if (frc::RobotBase::IsSimulation()) configureSignalHandlers();
  return frc::StartRobot<FunkyRobot>();
}
#endif